"""Positive + tampering tests for candidate data identity inspector (no machine)."""
import importlib.util
import hashlib
import json
import struct
import tempfile
from pathlib import Path

root=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('checker',root/'tools/check_multipose_provenance.py')
checker=importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)

with tempfile.TemporaryDirectory() as tmp:
    d=Path(tmp)
    cad=d/'part.step';cad.write_bytes(b'example cad')
    scan=d/'scan.scan';scan.write_bytes(b'example real scan')
    cad_digest=hashlib.sha256(cad.read_bytes()).hexdigest()
    scan_digest=hashlib.sha256(scan.read_bytes()).hexdigest()
    mat=[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.]
    params=[i*0.1 for i in range(17)]
    model_hash=hashlib.sha256(checker.PHYS_PREFIX+checker.normalized_doubles(params)).hexdigest()
    pose_hash=hashlib.sha256(checker.POSE_PREFIX+cad_digest.encode()+checker.normalized_doubles(mat)).hexdigest()
    model_id='CAL17_IMPORTED_SHA256_'+model_hash
    pose_id='POSE_CANDIDATE_SHA256_'+pose_hash
    text=f'; OfflineOnly: true\n; PartPoseId: {pose_id}\n; CalibrationId: {model_id}\n; Physical17Sha256: {model_hash}\nG00 X0 Y0 Z0 C0 B0\n'
    nc=d/'MP_ALL_POSES_CONNECTED_OFFLINE_ONLY_H3.nc';nc.write_text(text)
    manifest={
        'sourceCad':str(cad),'cadSha256':cad_digest,
        'sourceScan':str(scan),'realScanSha256':scan_digest,
        'cadToMachine':mat,'partPoseId':pose_id,
        'importedPhysical17Parameters':params,'physical17Sha256':model_hash,
        'calibrationId':model_id,
        'continuousNcSessions':[{'filename':nc.name,'sha256':hashlib.sha256(nc.read_bytes()).hexdigest()}],
    }
    manifest_file=d/'manifest.json';manifest_file.write_text(json.dumps(manifest))
    report=checker.audit(nc,manifest_file)
    assert report['importedSourceIdentityMatches'],report
    assert report['safeForHardwareExecution'] is False
    nc.write_text(text.replace('X0','X1'))
    assert not checker.audit(nc,manifest_file)['importedSourceIdentityMatches']
    nc.write_text(text)
    manifest['importedPhysical17Parameters'][0]=3.1
    manifest_file.write_text(json.dumps(manifest))
    assert not checker.audit(nc,manifest_file)['importedSourceIdentityMatches']
print('candidate provenance matching, NC tampering and model tampering PASS')
