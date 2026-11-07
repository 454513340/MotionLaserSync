#include "geometry/OrganizedCloudMesher.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

// Unlike assert(), CHECK is still active in MSVC/Release and -DNDEBUG builds.
#define CHECK(...) do { if (!(__VA_ARGS__)) { \
    std::cerr << "CHECK FAILED: " << #__VA_ARGS__ << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
    std::abort(); \
} } while (false)

using mls::geometry::Point3;
using mls::geometry::StripMeshOptions;
using mls::geometry::StripMeshResult;

struct Data {
    std::vector<Point3> points;
    std::vector<std::uint32_t> columns;
    std::vector<std::size_t> offsets;
    std::size_t width = 0;
    bool indexed = false;
    explicit Data(std::size_t w,std::size_t rows):width(w) {
        points.reserve(w*rows);
        offsets.push_back(0);
        for(std::size_t r=0;r<rows;++r) {
            for(std::size_t c=0;c<w;++c)
                points.push_back({0.1*double(c),0.3*double(r),-240.+double(c%17)*0.001});
            offsets.push_back(points.size());
        }
    }
    StripMeshResult mesh(const StripMeshOptions& options = {},bool cancel=false) const {
        return mls::geometry::buildTriangleStrips(points.size(),offsets.size()-1,width,
            [&](std::size_t i){return points[i];},
            [&](std::size_t r){return offsets[r];},
            [&](std::size_t i){return indexed?columns[i]:std::uint32_t(i%width);},
            options,[&]{return cancel;});
    }
};

static void testFlat640And1280() {
    for(const std::size_t w : {640u,1280u}) {
        Data d(w,4);
        auto m=d.mesh();
        CHECK(!m.invalidLayout && !m.cancelled);
        CHECK(m.triangles==2*3*(w-1));
        CHECK(m.stripCount()==3);
        CHECK(m.offsets.size()==4);
        CHECK(m.vertexIds.size()==2*4*w-2*w);
        CHECK(m.rowStride==1 && m.columnStride==1);
        CHECK(mls::geometry::guessFlatProfileWidth(d.points.size(),
            [&](std::size_t i){return d.points[i];})==int(w));
    }
}

static void testPreserveOriginalColumnHoles() {
    Data d(640,3);
    const std::size_t hole = 640+125;
    d.points.erase(d.points.begin()+std::ptrdiff_t(hole));
    d.indexed=true;
    d.offsets={0,640,1279,1919};
    for(std::size_t r=0;r<3;++r)
        for(std::size_t c=0;c<640;++c)
            if(!(r==1 && c==125))d.columns.push_back(std::uint32_t(c));
    const auto mesh=d.mesh();
    CHECK(!mesh.invalidLayout);
    CHECK(mesh.stripCount()==4); // each adjacent pair splits at missing column
    CHECK(mesh.triangles==2*2*(639-2)); // 2 row-pairs each loses 2 edges
    for(std::size_t s=0;s<mesh.stripCount();++s)
        for(std::size_t i=mesh.offsets[s]+2;i<mesh.offsets[s+1];i+=2)
            CHECK(d.columns[mesh.vertexIds[i]]==d.columns[mesh.vertexIds[i-2]]+1);
}

static void testBreakOnUnphysicalJumpAndNonfinite() {
    Data d(640,3);
    d.points[640+123].z+=120;
    d.points[640+156].z=std::numeric_limits<double>::quiet_NaN();
    auto m=d.mesh();
    CHECK(m.triangles<2*2*639);
    CHECK(m.rejectedSamples>0 || m.rejectedConnections>0);
    for(auto i:m.vertexIds)CHECK(mls::geometry::finitePoint(d.points[i]));
    // A very large inter-profile gap creates no connections through empty space.
    Data gap(640,3);
    for(std::size_t i=2*640;i<3*640;++i)gap.points[i].y+=150;
    auto g=gap.mesh();
    CHECK(g.triangles==2*639); // row 0->1 only; never bridges the 150mm gap
    CHECK(g.rejectedSamples>=640);
}

static void testManualLimitAndInvalidLayout() {
    Data d(640,2);
    StripMeshOptions s; s.maximumEdgeMm=0.2;
    CHECK(d.mesh(s).triangles==0);
    s.maximumEdgeMm=1.0;
    CHECK(d.mesh(s).triangles==2*639);
    d.indexed=true;
    d.columns.resize(1280);
    for(std::size_t i=0;i<1280;++i)d.columns[i]=i%640;
    d.columns[101]=d.columns[100];
    CHECK(d.mesh().invalidLayout);
    CHECK(d.mesh({},true).cancelled==false); // layout validation precedes cancellation
}

static void testCancellationAndPreviewBound() {
    Data d(1280,500);
    CHECK(d.mesh({},true).cancelled);
    StripMeshOptions o; o.maximumTriangles=40000;
    auto m=d.mesh(o);
    CHECK(m.reducedForPreview);
    CHECK(m.rowStride>1 && m.columnStride>1);
    CHECK(m.triangles<=o.maximumTriangles*1.1);
    CHECK(m.triangles>10000);
}

static void testRejectUnorderedXYZHeuristic() {
    std::vector<Point3> points;
    points.reserve(1280*4);
    std::uint32_t state=0x1f123bb5u;
    for(int i=0;i<1280*4;++i) {
        auto next=[&](){state=1664525u*state+1013904223u;return double(state%100000)/1000.;};
        points.push_back({next(),next(),next()});
    }
    CHECK(mls::geometry::guessFlatProfileWidth(points.size(),
        [&](std::size_t i){return points[i];})==0);
}

int main() {
    testFlat640And1280();
    testPreserveOriginalColumnHoles();
    testBreakOnUnphysicalJumpAndNonfinite();
    testManualLimitAndInvalidLayout();
    testCancellationAndPreviewBound();
    testRejectUnorderedXYZHeuristic();
    std::cout<<"organized_cloud_mesh_tests: PASS\n";
}
