#pragma once

// Triangle-strip topology for imported, organized laser profiles.
// This geometry module has no dependency on Qt, VTK or a hardware SDK and is
// deliberately not called by the live-acquisition rendering path.
//
// A row contains samples in increasing ORIGINAL sensor-column order. Missing
// samples must remain missing: callers pass their original column index rather
// than pretending that filtered XYZ points form consecutive columns.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace mls::geometry {

struct Point3 {
    double x = 0, y = 0, z = 0;
};

struct StripMeshOptions {
    double maximumEdgeMm = 0.0; // 0 = robust automatic, separate X/profile thresholds
    std::size_t maximumTriangles = 12000000; // preview memory guard, never modifies input data
};

struct StripMeshResult {
    // offsets[0]=0; strip i is vertexIds[offsets[i] .. offsets[i+1]).
    // Consecutive pairs are (rowA,columnJ),(rowB,columnJ), forming strips.
    std::vector<std::uint32_t> vertexIds;
    std::vector<std::size_t> offsets{0};
    std::size_t triangles = 0;
    std::size_t rejectedConnections = 0;
    std::size_t rejectedSamples = 0;
    std::size_t rows = 0;
    std::size_t columns = 0;
    std::size_t rowStride = 1;
    std::size_t columnStride = 1;
    double profileEdgeLimitMm = 0;
    double betweenEdgeLimitMm = 0;
    bool reducedForPreview = false;
    bool cancelled = false;
    bool invalidLayout = false;
    std::size_t stripCount() const { return offsets.size() - 1; }
};

inline bool finitePoint(const Point3& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
inline double squaredDistance(const Point3& a, const Point3& b) {
    const double x=a.x-b.x, y=a.y-b.y, z=a.z-b.z;
    return x*x+y*y+z*z;
}
inline double squaredTwiceArea(const Point3& a,const Point3& b,const Point3& c) {
    const double ux=b.x-a.x,uy=b.y-a.y,uz=b.z-a.z;
    const double vx=c.x-a.x,vy=c.y-a.y,vz=c.z-a.z;
    const double x=uy*vz-uz*vy,y=uz*vx-ux*vz,z=ux*vy-uy*vx;
    return x*x+y*y+z*z;
}
inline double median(std::vector<double>& samples) {
    if (samples.empty()) return 0;
    auto middle=samples.begin()+std::ptrdiff_t(samples.size()/2);
    std::nth_element(samples.begin(),middle,samples.end());
    return *middle;
}

// Requires getPoint(i) -> Point3. It is a *conservative heuristic*, not a
// replacement for indexed profile metadata in the .scan archive. If the XYZ
// file has been re-sorted or filtered, automatic meshing must be refused.
template<class PointAt>
int guessFlatProfileWidth(std::size_t count, PointAt getPoint) {
    int best=0;
    double bestScore=std::numeric_limits<double>::infinity();
    for (const int width : {1280,640}) {
        const std::size_t w=std::size_t(width);
        if (count < w*2 || count%w) continue;
        const std::size_t rows=count/w;
        std::vector<double> along,cross;
        int interiorSeamCount=0, checkedSeams=0;
        std::vector<double> rowExtents;
        const std::size_t rowStep=std::max<std::size_t>(1,rows/12);
        for (std::size_t row=0;row<rows;row+=rowStep) {
            const std::size_t start=row*w;
            Point3 minimum{1e300,1e300,1e300},maximum{-1e300,-1e300,-1e300};
            for (std::size_t col=0;col<w;col+=std::max<std::size_t>(1,w/96)) {
                const auto p=getPoint(start+col);
                if(!finitePoint(p))continue;
                minimum.x=std::min(minimum.x,p.x);minimum.y=std::min(minimum.y,p.y);
                minimum.z=std::min(minimum.z,p.z);
                maximum.x=std::max(maximum.x,p.x);maximum.y=std::max(maximum.y,p.y);
                maximum.z=std::max(maximum.z,p.z);
            }
            if(minimum.x<1e299)rowExtents.push_back(std::sqrt(squaredDistance(minimum,maximum)));
            for (std::size_t col=0;col+1<w;col+=std::max<std::size_t>(1,w/96)) {
                const auto a=getPoint(start+col),b=getPoint(start+col+1);
                if(finitePoint(a)&&finitePoint(b))along.push_back(std::sqrt(squaredDistance(a,b)));
            }
            if (row+1<rows) {
                for (std::size_t col=0;col<w;col+=std::max<std::size_t>(1,w/96)) {
                    const auto a=getPoint(start+col),b=getPoint(start+w+col);
                    if(finitePoint(a)&&finitePoint(b))cross.push_back(std::sqrt(squaredDistance(a,b)));
                }
            }
        }
        const double within=median(along),between=median(cross);
        const double span=median(rowExtents);
        // Random XYZ data can be divisible by 640/1280 by coincidence.
        // A true scanning line must vary smoothly across a spatial span that
        // is much larger than one neighboring sample step.
        if (!(within>1.e-7) || !(between>=0) || cross.empty() ||
            !(span>within*20.0)) continue;
        // A spurious 1280-wide row made of two true 640-wide rows has a
        // conspicuous internal 640-column seam, even when the cloud is flat.
        for(std::size_t row=0;row<rows;row+=rowStep) {
            const auto a=getPoint(row*w+w/2-1),b=getPoint(row*w+w/2);
            if(!finitePoint(a)||!finitePoint(b))continue;
            ++checkedSeams;
            if(std::sqrt(squaredDistance(a,b))>std::max(0.25,within*20.0))++interiorSeamCount;
        }
        const double ratio=between/std::max(within,1.e-7);
        if(ratio>200.0)continue; // likely half a profile, random XYZ or changed point order
        const double seamPenalty=checkedSeams?30.0*interiorSeamCount/checkedSeams:0.0;
        const double score=std::log1p(ratio)+seamPenalty;
        if (score<bestScore) {bestScore=score;best=width;}
    }
    return best;
}

// getRowStart(row) is an offset into the input point array (rows+1 offsets).
// getColumn(i) is the original sensor column (0..width-1), strictly increasing
// inside each row. For plain PLY/CSV: getRowStart(r)=r*width and
// getColumn(i)=i%width. A missing sample breaks a strip rather than shifting
// all later indices and generating diagonal cross-seam faces.
// Cancellation can be supplied as [&]{return cancelled.load();}.
template<class PointAt,class RowStart,class ColumnAt,class Cancelled>
StripMeshResult buildTriangleStrips(std::size_t pointCount, std::size_t rows,
        std::size_t width, PointAt getPoint, RowStart getRowStart,
        ColumnAt getColumn, const StripMeshOptions& options, Cancelled cancelled) {
    StripMeshResult out;
    out.rows=rows;out.columns=width;
    if(rows<2||width<2||pointCount<4||pointCount>std::uint64_t(std::numeric_limits<std::uint32_t>::max()))return out;
    if(getRowStart(0)!=0||getRowStart(rows)!=pointCount) {out.invalidLayout=true;return out;}
    for(std::size_t r=0;r<rows;++r) {
        const auto a=getRowStart(r),b=getRowStart(r+1);
        if(a>b||b>pointCount) {out.invalidLayout=true;return out;}
        std::size_t previous=0;
        for(std::size_t i=a;i<b;++i) {
            const auto c=std::size_t(getColumn(i));
            if(c>=width||(i>a && c<=previous)) {out.invalidLayout=true;return out;}
            previous=c;
        }
    }
    if(cancelled()){out.cancelled=true;return out;}
    // Use actual within-profile and between-profile lengths independently;
    // the two directions often differ by one or two orders of magnitude.
    std::vector<double> along,across;
    along.reserve(8192);across.reserve(8192);
    const std::size_t rowSampling=std::max<std::size_t>(1,rows/48);
    for(std::size_t row=0;row<rows;row+=rowSampling) {
        if(cancelled()){out.cancelled=true;return out;}
        const auto start=getRowStart(row),end=getRowStart(row+1);
        const std::size_t step=std::max<std::size_t>(1,(end-start)/96);
        for(std::size_t i=start;i+1<end;i+=step) {
            if(getColumn(i+1)!=getColumn(i)+1)continue;
            const auto a=getPoint(i),b=getPoint(i+1);
            if(finitePoint(a)&&finitePoint(b))along.push_back(std::sqrt(squaredDistance(a,b)));
        }
        if(row+1>=rows)continue;
        auto a=start,b=getRowStart(row+1);
        const auto ae=end,be=getRowStart(row+2);
        std::size_t seen=0;
        while(a<ae&&b<be) {
            const auto ca=getColumn(a),cb=getColumn(b);
            if(ca<cb){++a;continue;}
            if(ca>cb){++b;continue;}
            if(seen++ % std::max<std::size_t>(1,width/96)==0) {
                const auto pa=getPoint(a),pb=getPoint(b);
                if(finitePoint(pa)&&finitePoint(pb))across.push_back(std::sqrt(squaredDistance(pa,pb)));
            }
            ++a;++b;
        }
    }
    const double alongMedian=median(along),crossMedian=median(across);
    const bool manual=options.maximumEdgeMm>0&&std::isfinite(options.maximumEdgeMm);
    // The physical cap prevents a series of distinct/rotated scan passes from
    // being stitched across a very large gap when the small sample's median
    // itself is inflated. Operators can explicitly override it if needed.
    constexpr double kAutoMaximumEdgeMm = 25.0;
    out.profileEdgeLimitMm=manual?options.maximumEdgeMm:
        std::min(kAutoMaximumEdgeMm,std::max(0.05,alongMedian*5.0));
    out.betweenEdgeLimitMm=manual?options.maximumEdgeMm:
        std::min(kAutoMaximumEdgeMm,std::max(0.05,crossMedian*5.0));
    const double diagLimit=manual?options.maximumEdgeMm:
        std::hypot(out.profileEdgeLimitMm,out.betweenEdgeLimitMm)*1.2;

    const double estimate=2.0*double(rows-1)*double(width-1);
    if(options.maximumTriangles>0 && estimate>double(options.maximumTriangles)) {
        // A preview must not consume unbounded GPU/CPU connectivity memory.
        // Original imported points are kept intact; only the MESH is sampled.
        const auto step=std::size_t(std::ceil(std::sqrt(estimate/double(options.maximumTriangles))));
        out.rowStride=std::max<std::size_t>(1,step);
        out.columnStride=std::max<std::size_t>(1,step);
        out.reducedForPreview=true;
    }
    const double alongLimitSq=out.profileEdgeLimitMm*out.profileEdgeLimitMm;
    const double acrossLimitSq=out.betweenEdgeLimitMm*out.betweenEdgeLimitMm;
    const double diagonalLimitSq=diagLimit*diagLimit;
    std::vector<std::uint32_t> run;
    run.reserve(std::min<std::size_t>(width*2,8192));
    const auto flush=[&] {
        if(run.size()>=4) {
            out.triangles+=run.size()-2;
            out.vertexIds.insert(out.vertexIds.end(),run.begin(),run.end());
            out.offsets.push_back(out.vertexIds.size());
        }
        run.clear();
    };
    for(std::size_t row=0;row+out.rowStride<rows;row+=out.rowStride) {
        if(cancelled()){out.cancelled=true;out.vertexIds.clear();out.offsets.assign(1,0);out.triangles=0;return out;}
        auto a=getRowStart(row),b=getRowStart(row+out.rowStride);
        const auto ae=getRowStart(row+1),be=getRowStart(row+out.rowStride+1);
        std::size_t lastCol=width,prevA=0,prevB=0;
        run.clear();
        while(a<ae&&b<be) {
            const auto ca=std::size_t(getColumn(a)),cb=std::size_t(getColumn(b));
            if(ca<cb){++a;continue;}
            if(ca>cb){++b;continue;}
            const auto col=ca;
            if(col%out.columnStride!=0 && col+1!=width){++a;++b;continue;}
            const auto p=getPoint(a),q=getPoint(b);
            const double crossSq=squaredDistance(p,q);
            if(!finitePoint(p)||!finitePoint(q)||crossSq>
                    acrossLimitSq*double(out.rowStride*out.rowStride)) {
                ++out.rejectedSamples;flush();lastCol=width;
                ++a;++b;continue;
            }
            if(lastCol!=width) {
                const auto u=getPoint(prevA),v=getPoint(prevB);
                const double delta=double(col-lastCol);
                // Missing original columns must terminate a strip. Only a
                // deliberately chosen preview stride may skip columns.
                const bool neighboring=col>lastCol &&
                    (col-lastCol==out.columnStride ||
                     (out.columnStride>1 && col+1==width && col-lastCol<out.columnStride));
                const bool connected=neighboring &&
                    squaredDistance(u,p)<=alongLimitSq*delta*delta &&
                    squaredDistance(v,q)<=alongLimitSq*delta*delta &&
                    squaredDistance(v,p)<=diagonalLimitSq*std::max(delta*delta,double(out.rowStride*out.rowStride)) &&
                    squaredDistance(u,q)<=diagonalLimitSq*std::max(delta*delta,double(out.rowStride*out.rowStride)) &&
                    squaredTwiceArea(u,v,p)>1.e-14 &&
                    squaredTwiceArea(v,p,q)>1.e-14;
                if(!connected){++out.rejectedConnections;flush();}
            }
            run.push_back(std::uint32_t(a));run.push_back(std::uint32_t(b));
            lastCol=col;prevA=a;prevB=b;
            ++a;++b;
        }
        flush();
    }
    return out;
}

} // namespace mls::geometry
