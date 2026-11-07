# Conditional OfflineOnly NC release tests

Run the following in a Python + C++17 environment; no machine motion is issued:

```
python tests/gcode/test_offline_nc_conditional_release_contract.py
python tests/gcode/test_nc_probe_fcl_new_flow.py
python tests/gcode/test_real_run_preflight_audit.py
python tests/gcode/test_imported_model_provenance_contract.py
python tests/gcode/test_gcode_settings_live_entry_contract.py
python tests/test_live_nc_deployment_contract.py
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tests/gcode/test_hardware_nc_probe_policy.cpp -o test_probe
./test_probe
```

This is not a Windows/Qt6 GUI build or a hardware acceptance test. Without the real on-site safety/fixture readback provider, the machine release verifier fails closed.
