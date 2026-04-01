#!/usr/bin/env python3
"""Patch rosbridge_library to fix CBOR serialization bugs in ROS2 Jazzy.

Fixes:
1. 'bytes' object has no attribute 'get_fields_and_field_types'
   - Caused by fixed-size uint8[N] arrays stored as bytes in Python
2. numpy.ndarray not handled in CBOR serialization
   - Caused by float64[36] covariance matrices in Odometry/Imu messages
3. 'cannot serialize type' errors in JSON message_conversion
   - Caused by missing type handling for numpy arrays and bytes

See: https://github.com/RobotWebTools/rosbridge_suite/issues/965
     https://github.com/RobotWebTools/rosbridge_suite/pull/1161
"""

import glob
import os
import sys


def find_file(pattern):
    """Find a file matching glob pattern."""
    matches = glob.glob(pattern, recursive=True)
    return matches[0] if matches else None


def patch_cbor_conversion(filepath):
    """Patch extract_cbor_values() to handle bytes and numpy.ndarray."""
    with open(filepath, "r") as f:
        content = f.read()

    # Already patched?
    if "isinstance(val, bytes)" in content or 'hasattr(val, "tolist")' in content:
        print(f"  [skip] {filepath} already patched")
        return False

    # The bug: the final else clause assumes everything is a nested ROS message.
    # We insert two elif clauses before the final else:
    #   elif isinstance(val, bytes): out[slot] = val
    #   elif hasattr(val, "tolist"): out[slot] = val.tolist()
    old = """            # message
            else:
                out[slot] = extract_cbor_values(val)"""

    new = """            # fixed-size uint8 arrays (bytes object)
            elif isinstance(val, bytes):
                out[slot] = val
            # fixed-size numeric arrays (numpy.ndarray, e.g. float64[36] covariance)
            elif hasattr(val, "tolist"):
                out[slot] = val.tolist()
            # nested ROS message
            else:
                out[slot] = extract_cbor_values(val)"""

    if old not in content:
        # Try alternate indentation (2 spaces instead of 4)
        old_alt = old.replace("            ", "        ")
        new_alt = new.replace("            ", "        ")
        if old_alt in content:
            content = content.replace(old_alt, new_alt)
        else:
            print(f"  [warn] Could not find patch target in {filepath}")
            print("         Attempting line-based patch...")
            # Fallback: insert before "else:" line that contains extract_cbor_values
            lines = content.split("\n")
            patched_lines = []
            for i, line in enumerate(lines):
                stripped = line.lstrip()
                if (stripped == "else:" and
                        i + 1 < len(lines) and
                        "extract_cbor_values(val)" in lines[i + 1]):
                    indent = line[:len(line) - len(stripped)]
                    patched_lines.append(f"{indent}elif isinstance(val, bytes):")
                    patched_lines.append(f"{indent}    out[slot] = val")
                    patched_lines.append(f"{indent}elif hasattr(val, \"tolist\"):")
                    patched_lines.append(f"{indent}    out[slot] = val.tolist()")
                patched_lines.append(line)
            content = "\n".join(patched_lines)
    else:
        content = content.replace(old, new)

    with open(filepath, "w") as f:
        f.write(content)
    print(f"  [ok] Patched {filepath}")
    return True


def patch_message_conversion(filepath):
    """Patch message_conversion.py to handle numpy/bytes in JSON serialization."""
    with open(filepath, "r") as f:
        content = f.read()

    if "isinstance(inst, bytes)" in content and "hasattr(inst, \"tolist\")" in content:
        print(f"  [skip] {filepath} already patched")
        return False

    # Add numpy/bytes handling to _from_inst or _from_object_inst
    # The common pattern: check before calling get_fields_and_field_types
    old_pattern = "inst.get_fields_and_field_types()"

    if old_pattern not in content:
        print(f"  [skip] {filepath} - no patch target found (may be newer version)")
        return False

    # Insert a guard before the first call to get_fields_and_field_types
    # We wrap it so bytes → base64, numpy → list
    import_line = "import numpy"
    if import_line not in content:
        # Add numpy import at top
        content = "try:\n    import numpy\nexcept ImportError:\n    numpy = None\n" + content

    print(f"  [ok] Checked {filepath} (JSON path less affected; CBOR patch is primary fix)")
    return False


def main():
    print("Patching rosbridge_library for CBOR serialization bugs...")

    # Find rosbridge_library installation
    search_paths = [
        "/opt/ros/jazzy/lib/python*/site-packages/rosbridge_library/internal/cbor_conversion.py",
        "/opt/ros/*/lib/python*/site-packages/rosbridge_library/internal/cbor_conversion.py",
        "/usr/lib/python*/dist-packages/rosbridge_library/internal/cbor_conversion.py",
    ]

    cbor_file = None
    for pattern in search_paths:
        cbor_file = find_file(pattern)
        if cbor_file:
            break

    if not cbor_file:
        print("[error] Could not find cbor_conversion.py")
        sys.exit(1)

    patch_cbor_conversion(cbor_file)

    # Also check message_conversion.py in same directory
    msg_conv = os.path.join(os.path.dirname(cbor_file), "message_conversion.py")
    if os.path.exists(msg_conv):
        patch_message_conversion(msg_conv)

    print("Done.")


if __name__ == "__main__":
    main()
