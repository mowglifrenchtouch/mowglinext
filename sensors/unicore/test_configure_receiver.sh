#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/configure_receiver.sh"

failures=0

assert_eq() {
  local label="${1:?assert_eq: missing label}"
  local expected="${2-}"
  local actual="${3-}"

  if [ "$expected" != "$actual" ]; then
    echo "FAIL: $label"
    echo "  expected: $expected"
    echo "  actual:   $actual"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

assert_contains() {
  local label="${1:?assert_contains: missing label}"
  local needle="${2:?assert_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" != *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  missing: $needle"
    echo "  actual:  $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

assert_not_contains() {
  local label="${1:?assert_not_contains: missing label}"
  local needle="${2:?assert_not_contains: missing needle}"
  local haystack="${3-}"

  if [[ "$haystack" == *"$needle"* ]]; then
    echo "FAIL: $label"
    echo "  unexpected: $needle"
    echo "  actual:     $haystack"
    failures=$((failures + 1))
  else
    echo "PASS: $label"
  fi
}

reset_mocks() {
  COMMAND_LOG=""
  STTY_LOG=""
  CURRENT_BAUD=""
  MOCK_MODEL="unknown"
  MOCK_RESPONDING_BAUDS=""
}

reset_profile_env() {
  UNICORE_PROFILE="normal"
  UNICORE_OUTPUT_FORMAT="ascii"
  UNICORE_SIGNALGROUP_OVERRIDE=""
  UNICORE_MAIN_LOG_PERIOD=""
  UNICORE_BESTNAV_LOG_PERIOD=""
  UNICORE_DIAGNOSTIC_LOG_PERIOD=""
  UNICORE_SATELLITE_LOG_PERIOD=""
  UNICORE_RF_LOG_PERIOD=""
  UNICORE_RAW_LOG_PERIOD=""
  UNICORE_ENABLE_SATELLITES=""
  UNICORE_ENABLE_RF=""
  UNICORE_ENABLE_JAMMING=""
  UNICORE_ENABLE_HARDWARE=""
  UNICORE_ENABLE_RAW_OBSERVATIONS=""
}

require_serial_port() { :; }
wait_for_serial_port() { :; }
open_serial() { :; }
close_serial() { :; }
drain_serial() { :; }
sleep() { :; }

serial_set_baud() {
  local _port="${1:?serial_set_baud: missing port}"
  local baud="${2:?serial_set_baud: missing baud}"

  CURRENT_BAUD="$baud"
  STTY_LOG+="${baud}"$'\n'
}

query_receiver_identification() {
  case " $MOCK_RESPONDING_BAUDS " in
    *" $CURRENT_BAUD "*) ;;
    *) return 0 ;;
  esac

  case "$MOCK_MODEL" in
    UM980) printf '%s\n' '#VERSIONA,"UM980","R4.10Build15434"' ;;
    UM982) printf '%s\n' '#VERSIONA,"UM982","R4.10Build15434"' ;;
    *) printf '%s\n' '#VERSIONA,"UNKNOWN","R4.10Build15434"' ;;
  esac
}

send_serial_command() {
  local command="${1:?send_serial_command: missing command}"

  COMMAND_LOG+="${command}"$'\n'
  if [[ "$command" =~ ^CONFIG[[:space:]]+COM1[[:space:]]+([0-9]+)$ ]]; then
    CURRENT_BAUD="${BASH_REMATCH[1]}"
  fi
}

run_scenario() {
  local label="${1:?run_scenario: missing label}"
  local port="/dev/mock"
  local preferred_baud="${2:?run_scenario: missing preferred baud}"
  local model="${3:?run_scenario: missing model}"
  local responding_bauds="${4:?run_scenario: missing responding bauds}"
  local expected_rc="${5:?run_scenario: missing expected rc}"
  local rc=0

  echo ""
  echo "Scenario: $label"
  reset_mocks
  MOCK_MODEL="$model"
  MOCK_RESPONDING_BAUDS="$responding_bauds"

  if main "$port" "$preferred_baud"; then
    rc=0
  else
    rc=$?
  fi

  assert_eq "$label rc" "$expected_rc" "$rc"
}

run_scenario "UM980 from 460800" "460800" "UM980" "460800 921600" "0"
assert_contains "UM980 switches to 921600" "CONFIG COM1 921600" "$COMMAND_LOG"
assert_contains "UM980 uses SIGNALGROUP 2" "CONFIG SIGNALGROUP 2" "$COMMAND_LOG"
assert_contains "UM980 saves config" "SAVECONFIG" "$COMMAND_LOG"

run_scenario "UM982 already at 921600" "921600" "UM982" "921600" "0"
assert_contains "UM982 uses SIGNALGROUP 3 6" "CONFIG SIGNALGROUP 3 6" "$COMMAND_LOG"
assert_eq "UM982 does not resend baud config" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG COM1 921600" || true)"

run_scenario "Unknown model is rejected" "115200" "unknown" "115200" "1"
assert_eq "Unknown model skips saveconfig" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "SAVECONFIG" || true)"
assert_eq "Unknown model skips signalgroup" "" "$(printf '%s' "$COMMAND_LOG" | grep -F "CONFIG SIGNALGROUP" || true)"

reset_profile_env
UNICORE_PROFILE="normal"
unicore_apply_profile_defaults
NORMAL_LOGS="$(build_log_commands)"
assert_not_contains "normal excludes BESTSATA" "BESTSATA" "$NORMAL_LOGS"
assert_not_contains "normal excludes SATSINFOA" "SATSINFOA" "$NORMAL_LOGS"
assert_not_contains "normal excludes AGCA" "AGCA" "$NORMAL_LOGS"
assert_not_contains "normal excludes JAMSTATUSA" "JAMSTATUSA" "$NORMAL_LOGS"
assert_contains "normal keeps PVTSLNA" "PVTSLNA" "$NORMAL_LOGS"
assert_not_contains "normal excludes PVTSLNB" "PVTSLNB" "$NORMAL_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
unicore_apply_profile_defaults
DEBUG_LOGS="$(build_log_commands)"
assert_contains "debug includes BESTSATA" "BESTSATA" "$DEBUG_LOGS"
assert_contains "debug includes SATSINFOA" "SATSINFOA" "$DEBUG_LOGS"
assert_contains "debug includes AGCA" "AGCA" "$DEBUG_LOGS"
assert_contains "debug includes HWSTATUSA" "HWSTATUSA" "$DEBUG_LOGS"
assert_contains "debug includes JAMSTATUSA" "JAMSTATUSA" "$DEBUG_LOGS"
assert_contains "debug includes FREQJAMSTATUSA" "FREQJAMSTATUSA" "$DEBUG_LOGS"
assert_not_contains "debug ascii excludes BESTSATB" "BESTSATB" "$DEBUG_LOGS"
assert_not_contains "debug ascii excludes PVTSLNB" "PVTSLNB" "$DEBUG_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
UNICORE_OUTPUT_FORMAT="hybrid"
unicore_apply_profile_defaults
DEBUG_HYBRID_LOGS="$(build_log_commands)"
assert_contains "debug hybrid includes PVTSLNA" "PVTSLNA" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes PVTSLNB" "PVTSLNB" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes BESTSATA" "BESTSATA" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid includes BESTSATB" "BESTSATB" "$DEBUG_HYBRID_LOGS"
assert_contains "debug hybrid keeps GPGGA" "GPGGA" "$DEBUG_HYBRID_LOGS"

reset_profile_env
UNICORE_PROFILE="debug"
UNICORE_OUTPUT_FORMAT="binary"
unicore_apply_profile_defaults
DEBUG_BINARY_LOGS="$(build_log_commands)"
assert_not_contains "debug binary excludes PVTSLNA" "PVTSLNA" "$DEBUG_BINARY_LOGS"
assert_contains "debug binary includes PVTSLNB" "PVTSLNB" "$DEBUG_BINARY_LOGS"
assert_not_contains "debug binary excludes BESTSATA" "BESTSATA" "$DEBUG_BINARY_LOGS"
assert_contains "debug binary includes BESTSATB" "BESTSATB" "$DEBUG_BINARY_LOGS"
assert_not_contains "debug binary excludes GPGGA" "GPGGA" "$DEBUG_BINARY_LOGS"

reset_profile_env
UNICORE_PROFILE="survey"
unicore_apply_profile_defaults
SURVEY_LOGS="$(build_log_commands)"
assert_not_contains "survey excludes raw observations by default" "OBSVMCMP" "$SURVEY_LOGS"
assert_eq "survey main log period default" "1" "$UNICORE_MAIN_LOG_PERIOD"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
SURVEY_RAW_ASCII_LOGS="$(build_log_commands)"
assert_contains "survey raw ascii includes OBSVMCMPA" "OBSVMCMPA" "$SURVEY_RAW_ASCII_LOGS"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_OUTPUT_FORMAT="hybrid"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
SURVEY_RAW_BINARY_LOGS="$(build_log_commands)"
assert_contains "survey raw hybrid includes OBSVMCMPB" "OBSVMCMPB" "$SURVEY_RAW_BINARY_LOGS"
assert_contains "survey raw hybrid keeps OBSVMCMPA" "OBSVMCMPA" "$SURVEY_RAW_BINARY_LOGS"

reset_profile_env
UNICORE_PROFILE="normal"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
assert_eq "normal refuses raw observations" "false" "$UNICORE_ENABLE_RAW_OBSERVATIONS"
assert_not_contains "normal raw refusal emits no OBSVMCMP" "OBSVMCMP" "$(build_log_commands)"

reset_profile_env
UNICORE_PROFILE="survey"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
UNICORE_RAW_LOG_PERIOD="0.2"
unicore_apply_profile_defaults
assert_eq "survey raw period is clamped" "1" "$UNICORE_RAW_LOG_PERIOD"

reset_profile_env
UNICORE_PROFILE="high_precision"
unicore_apply_profile_defaults
HIGH_PRECISION_COMMANDS="$(build_profile_commands)"
assert_contains "high_precision enables PVTALG MULTI" "CONFIG PVTALG MULTI" "$HIGH_PRECISION_COMMANDS"
assert_contains "high_precision enables RTCMDECAUTO" "CONFIG RTCMDECAUTO ENABLE" "$HIGH_PRECISION_COMMANDS"

reset_profile_env
UNICORE_PROFILE="high_precision"
UNICORE_OUTPUT_FORMAT="binary"
UNICORE_ENABLE_RAW_OBSERVATIONS="true"
unicore_apply_profile_defaults
HIGH_PRECISION_BINARY_LOGS="$(build_log_commands)"
assert_contains "high_precision binary includes BESTNAVB" "BESTNAVB" "$HIGH_PRECISION_BINARY_LOGS"
assert_contains "high_precision binary includes OBSVMCMPB" "OBSVMCMPB" "$HIGH_PRECISION_BINARY_LOGS"
assert_not_contains "high_precision binary excludes BESTNAVA" "BESTNAVA" "$HIGH_PRECISION_BINARY_LOGS"

reset_profile_env
UNICORE_OUTPUT_FORMAT="hybrid"
unicore_apply_profile_defaults
assert_eq "hybrid output format is preserved" "hybrid" "$UNICORE_OUTPUT_FORMAT"
assert_eq "hybrid enables binary transport" "true" "$(unicore_binary_enabled_from_output_format "$UNICORE_OUTPUT_FORMAT")"

reset_profile_env
UNICORE_OUTPUT_FORMAT="broken"
unicore_apply_profile_defaults
assert_eq "invalid output format falls back to ascii" "ascii" "$UNICORE_OUTPUT_FORMAT"
assert_eq "ascii keeps binary transport disabled" "false" "$(unicore_binary_enabled_from_output_format "$UNICORE_OUTPUT_FORMAT")"

if [ "$failures" -ne 0 ]; then
  echo ""
  echo "$failures test(s) failed."
  exit 1
fi

echo ""
echo "All tests passed."
