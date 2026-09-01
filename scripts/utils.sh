# ===========================================================================
# FUNCTION: ensure_wifi_connection
# PURPOSE:  Safely checks the current Wi-Fi network and connects to a target
#           SSID only if it is not already connected. Supports Linux and macOS.
# USAGE:    ensure_wifi_connection "SSID" ["PASSWORD"] ["USE_ETH (true/false)"]
# ===========================================================================

ensure_wifi_connection() {
  # 1. Capture and localize arguments
  local target_ssid="${1:-}"
  local password="${2:-}"
  local use_eth="${3:-false}"

  # 2. Skip logic if using Ethernet or if no SSID is provided
  if [ "$use_eth" = "true" ]; then
    echo -e "\e[34m[INFO]\e[0m Ethernet mode active. Skipping Wi-Fi connection."
    return 0
  fi

  if [ -z "$target_ssid" ]; then
    echo -e "\e[33m[WARNING]\e[0m No SSID provided. Skipping Wi-Fi auto-connect."
    return 0
  fi

  # 3. Linux (nmcli) Implementation
  if command -v nmcli >/dev/null 2>&1; then
    local current_ssid
    # Safe parsing: || true ensures set -e doesn't crash the script if no wifi is active
    current_ssid=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2 || true)
    
    if [ "$current_ssid" = "$target_ssid" ]; then
      echo -e "\e[32m[INFO]\e[0m Already connected to Wi-Fi: ${target_ssid}"
      return 0
    fi

    echo -e "\e[34m[INFO]\e[0m Attempting to connect to Wi-Fi: ${target_ssid}..."
    if [ -n "$password" ]; then
      nmcli device wifi connect "$target_ssid" password "$password" >/dev/null 2>&1 || true
    else
      nmcli device wifi connect "$target_ssid" >/dev/null 2>&1 || true
    fi

  # 4. macOS (networksetup) Implementation
  elif command -v networksetup >/dev/null 2>&1; then
    local wifi_iface current_ssid
    
    # Dynamically find the Wi-Fi hardware port (usually en0)
    wifi_iface=$(networksetup -listallhardwareports 2>/dev/null | awk '/Hardware Port: Wi-Fi/{getline; print $2}' || true)
    
    if [ -z "$wifi_iface" ]; then
      echo -e "\e[31m[ERROR]\e[0m Could not detect a Wi-Fi interface on this Mac."
      return 1
    fi

    # Check current network
    current_ssid=$(networksetup -getairportnetwork "$wifi_iface" 2>/dev/null | awk -F': ' '{print $2}' || true)
    
    if [ "$current_ssid" = "$target_ssid" ]; then
      echo -e "\e[32m[INFO]\e[0m Already connected to Wi-Fi: ${target_ssid}"
      return 0
    fi

    echo -e "\e[34m[INFO]\e[0m Attempting to connect to Wi-Fi: ${target_ssid}..."
    if [ -n "$password" ]; then
      networksetup -setairportnetwork "$wifi_iface" "$target_ssid" "$password" >/dev/null 2>&1 || true
    else
      networksetup -setairportnetwork "$wifi_iface" "$target_ssid" >/dev/null 2>&1 || true
    fi

  # 5. Unsupported OS
  else
    echo -e "\e[33m[WARNING]\e[0m OS not supported for auto-connect. Please switch to '${target_ssid}' manually."
  fi
}


# ===========================================================================
# FUNCTION: wait_for_ssh
# PURPOSE:  Polls a target over SSH until it responds or times out.
# USAGE:    wait_for_ssh "user@ip" [max_retries]
# ===========================================================================
wait_for_ssh() {
  local target="$1"
  local max_retries="${2:-30}" # Defaults to 30 retries if not provided
  local use_eth="${3:-false}"
  local retry_count=0

  if [ "$use_eth" = "true" ]; then
    echo -n "Waiting for SSH to ${target} over Ethernet... "
  else
    echo -n "Waiting for SSH to ${target} (connect to Wi-Fi manually if needed)... "
  fi

  # Loop until SSH succeeds or we hit the retry limit
  while ! ssh -q -o BatchMode=yes -o ConnectTimeout=2 -o StrictHostKeyChecking=accept-new "${target}" "echo ok" > /dev/null 2>&1; do
    sleep 2
    echo -n "."
    retry_count=$((retry_count + 1))
    
    if [ "$retry_count" -ge "$max_retries" ]; then
      echo ""
      echo -e "\e[31m[ERROR]\e[0m Timeout: Could not connect to ${target}."
      exit 1
    fi
  done

  echo ""
  echo -e "\e[32m[SUCCESS]\e[0m Connection established."
}