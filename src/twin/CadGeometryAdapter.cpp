#include "twin/CadGeometryAdapter.h"
#include <cmath>
#include <utility>
#include <exception>
#ifdef MLS_TWIN_HAVE_OCCT
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopoDS.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pnt.hxx>
#endif
namespace mls::twin {
bool CadGeometryAdapter::stepSupported() {
#ifdef MLS_TWIN_HAVE_OCCT
    return true;
#else
    return false;
#endif
}
CadImportResult CadGeometryAdapter::loadStep(const std::string& path,const CadImportOptions& options) {
    CadImportResult r;
#ifndef MLS_TWIN_HAVE_OCCT
    (void)path;(void)options;
    r.message="Open CASCADE is not installed/enabled. Install OCCT, reconfigure CMake; STEP import is unavailable (OBJ demo still works).";
#else
    try {
        if(!(std::isfinite(options.linearDeflectionMm)&&options.linearDeflectionMm>0&&
             std::isfinite(options.angularDeflectionRad)&&options.angularDeflectionRad>0)){
            r.message="Invalid tessellation parameters";return r;
        }
        // The program's geometry convention is millimetres. OCCT STEP transfer converts
        // source file units to the target system (mm) at import time.
        Interface_Static::SetCVal("xstep.cascade.unit","MM");
        STEPControl_Reader reader;
        if(reader.ReadFile(path.c_str())!=IFSelect_RetDone){r.message="STEP reader failed to open/parse file";return r;}
        if(reader.TransferRoots()<=0){r.message="No transferable STEP roots";return r;}
        const TopoDS_Shape shape=reader.OneShape();
        if(shape.IsNull()){r.message="Empty STEP shape";return r;}
        BRepMesh_IncrementalMesh mesher(shape,options.linearDeflectionMm,false,
                                        options.angularDeflectionRad,true);
        TriangleMesh mesh;mesh.source=path;
        std::uint32_t faceId=0;
        for(TopExp_Explorer it(shape,TopAbs_FACE);it.More();it.Next()){
            ++faceId;
            const TopoDS_Face face=TopoDS::Face(it.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation=BRep_Tool::Triangulation(face,location);
            if(triangulation.IsNull())continue;
            const auto transform=location.Transformation();
            for(Standard_Integer i=1;i<=triangulation->NbTriangles();++i){
                Standard_Integer ia,ib,ic;
                triangulation->Triangle(i).Get(ia,ib,ic);
                if(face.Orientation()==TopAbs_REVERSED)std::swap(ib,ic);
                const auto convert=[&](Standard_Integer idx){
                    const gp_Pnt p=triangulation->Node(idx).Transformed(transform);
                    return Vec3{p.X(),p.Y(),p.Z()};
                };
                const Vec3 a=convert(ia),b=convert(ib),c=convert(ic);
                if(length(cross(b-a,c-a))<1e-13)continue;
                mesh.triangles.push_back({a,b,c,faceId});
            }
        }
        mesh.rebuildBounds();
        if(mesh.empty()){r.message="STEP imported but has no valid triangles";return r;}
        r.ok=true;r.brepFaces=faceId;r.mesh=std::move(mesh);
        r.message="STEP B-Rep imported and triangulated in mm";
    }catch(const Standard_Failure& e){r.message=std::string("Open CASCADE: ")+e.GetMessageString();}
    catch(const std::exception& e){r.message=e.what();}
#endif
    return r;
}
} // namespace mls::twin
