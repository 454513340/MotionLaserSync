#!/usr/bin/env python3
"""Read-only, fail-closed audit of a V29 multi-pose NC and its experiment manifest.

No controller/sensor commands. Header and manifest fields are NOT authorizations;
this report is supporting evidence for a separate on-site commissioning process.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path


def audit(nc_path: Path, manifest_path: Path | None = None) -> dict:
    data = nc_path.read_bytes()
    sha = hashlib.sha256(data).hexdigest()
    txt = data.decode('utf-8', 'replace')
    lines = txt.splitlines()
    head: dict[str, str] = {}
    body = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith(';'):
            m = re.match(r'^;\s*([^:]+):\s*(.*?)\s*$', stripped)
            if m:
                head[m.group(1).strip().casefold()] = m.group(2).strip()
        elif stripped:
            body.append(stripped)
    def count(cmd: str) -> int:
        return sum(bool(re.match(r'^'+re.escape(cmd)+r'(?:\s|$)', row, re.I)) for row in body)
    blocks = []
    observations = []
    for name in ('offlineonly', 'registrationverified', 'simulationverified', 'initialjointanchored'):
        value = head.get(name, '(missing)')
        observations.append(f'{name}: {value}')
        if name == 'offlineonly':
            if value.lower() != 'false':
                blocks.append('NC remains OfflineOnly or omits an explicit non-offline declaration')
        elif name == 'initialjointanchored':
            # Import does not require the machine to already be at first G00.
            # Qualification of the live-to-G00 move occurs immediately before
            # physical motion, based on the live MPOS and the approved NC.
            observations.append('first G00 remains a positioning command; entry path is checked at run time')
        elif value.lower() != 'true':
            blocks.append(f'{name} is not verified')
    for name in ('partposeid', 'calibrationid'):
        value = head.get(name, '(missing)')
        if not value or value == '(missing)' or 'UNVERIFIED' in value.upper():
            blocks.append(f'{name} has no independently accepted provenance')

    first = next((row for row in body if re.match(r'^G0?0\b', row, re.I)), None)
    first_joints = None
    if first:
        words = dict((letter.upper(), float(value)) for letter, value in
                     re.findall(r'([XYZCB])\s*([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)', first))
        if all(k in words for k in 'XYZCB'):
            first_joints = [words[k] for k in 'XYZCB']
    if first_joints is None:
        blocks.append('first full-axis G00 machine reference not found')
    else:
        observations.append('initial G00 is present; its live MPOS-to-target trajectory must be checked before running')
    if head.get('hardwaretopology', '').upper() not in ('H1','H3'):
        blocks.append('hardware topology missing or invalid')
    if head.get('hardwaretopology', '').upper() == 'H3' and head.get('cmmr1edge', '') not in ('114', '115'):
        blocks.append('H3 CMM edge not declared 114/115')
    counts = {k: count(k) for k in ('M50','M51','M60','M61','M63','M30')}
    if counts['M50'] != counts['M51'] or counts['M50'] != counts['M60'] or counts['M50'] != counts['M61']:
        blocks.append('capture/stop/save/clear group count mismatch')
    if counts['M63'] != 1 or counts['M30'] != 1:
        blocks.append('missing or repeated terminal save barrier/program end')
    manifest_info = {}
    if manifest_path:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        session = manifest.get('continuousNcSession') or {}
        expected = session.get('sha256')
        manifest_info = {
            'expectedSha256': expected, 'ncSha256Matches': expected == sha,
            'hardwareAuthorized': manifest.get('hardwareAuthorized', None),
            'registrationVerified': manifest.get('registrationVerified', None),
            'opticsAcrossAnglesVerified': manifest.get('opticsAcrossAnglesVerified', None),
            'firstG00StartUnverified': session.get('firstG00StartUnverified', None),
            'interPoseTransfersFclSamplesValidated': session.get('interPoseTransfersFclSamplesValidated', None),
            'laserOffPoseConnections': session.get('laserOffPoseConnections', None)
        }
        if expected != sha:
            blocks.append('NC SHA256 does not match experiment manifest')
        if manifest.get('hardwareAuthorized') is not True:
            blocks.append('experiment manifest does not certify hardware execution')
        if manifest.get('registrationVerified') is not True:
            blocks.append('manifest explicitly states registration is not verified')
        if manifest.get('opticsAcrossAnglesVerified') is not True:
            blocks.append('manifest does not certify optics across B/C angles')
        if session.get('firstG00StartUnverified') is not False:
            observations.append('manifest has no live entry certification; this is a RUN-TIME prerequisite, not a file-import position requirement')
    else:
        blocks.append('no matching experiment manifest supplied')
    blocks.append('no independent live machine/FCL, interlock and H1/H3 electrical commissioning evidence in this read-only audit')
    return {
        'schema': 'MotionLaserSyncV29RealRunReadOnlyAudit-v2',
        'auditedUtc': datetime.now(timezone.utc).isoformat(),
        'ncFile': str(nc_path),
        'ncByteCount': len(data),
        'sha256': sha,
        'header': head,
        'firstG00': first_joints,
        'commandCounts': counts,
        'g05ScanGroups': count('G05 S1'),
        'g05NonScanTransfers': count('G05 S0'),
        'manifest': manifest_info,
        'observations': observations,
        'blockingReasons': blocks,
        'hardwareExecutionAuthorized': False,
        'readOnly': True
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nc', required=True, type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = audit(args.nc,args.manifest)
    serialized = json.dumps(result,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(serialized+'\n',encoding='utf-8')
    print(json.dumps({
        'ncSha256':result['sha256'],
        'programs':result['g05ScanGroups'],
        'transfers':result['g05NonScanTransfers'],
        'matchedManifestHash':result['manifest'].get('ncSha256Matches'),
        'blockingReasons':result['blockingReasons']
    },ensure_ascii=False,indent=2))
    # Explicit nonzero exit is useful in automated quality gates.
    raise SystemExit(2 if result['blockingReasons'] else 0)

if __name__=='__main__':
    main()
