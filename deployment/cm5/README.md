# CM5 passive deployment

The CM5 runs a Micro XRCE-DDS Agent and a separate ROS 2 Humble launch under systemd as `drone-runtime`. The Agent bridges one PX4 controller. The ROS stack publishes state, safety and mission telemetry. The command API stays available for compatibility and rejects requests in read-only mode. The passive manager starts in STANDBY and rejects operator control requests. It consumes telemetry with stage feedback disabled. The launch checks the local DDS graph for competing mission managers and PX4 command publishers before starting and while running. Use one dedicated domain for this stack. Graph checks depend on discovery and supervise only this launch. Restrict access to the DDS network and the host. [Repository audit](AUDIT.md)

## Select and flash the OS

Canonical lists CM5 support for Ubuntu 24.04 LTS and leaves the CM5 22.04 entry blank. Use **Ubuntu Server 24.04 ARM64 with a Jammy ARM64 LXD system container** on a carrier confirmed to support the image. Use native Jammy only with a carrier-vendor-supported image and evidence that its EEPROM, kernel, device tree and boot storage support your CM5. The installer requires Jammy userspace in either case. [Canonical board support matrix](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/), [ROS Humble Ubuntu installation](https://github.com/ros2/ros2_documentation/blob/humble/source/Installation/Ubuntu-Install-Debs.rst)

1. Record the carrier make/revision, CM5 RAM/eMMC variant, storage, supported EEPROM revision, OS image and kernel. Check Ethernet, USB and the intended UART against the carrier's supported overlays and pinmux. Use the vendor's board manual for boot controls and cable/power specifications.
2. For eMMC, put the carrier into its documented USB provisioning mode and use Raspberry Pi's current `rpiboot`/mass-storage procedure from the PC. Confirm disk identity and capacity before writing and verify the image. Eject, power down, restore normal boot mode and boot. For a CM5 Lite, use the supported microSD, NVMe or USB medium and the documented EEPROM boot order. For boot failure, check power, media, image checksum, boot order and boot LEDs/console. Re-enter USB provisioning to re-image eMMC. Follow the board's EEPROM recovery procedure if the bootloader requires repair. [Raspberry Pi Compute Module flashing and recovery](https://www.raspberrypi.com/documentation/computers/compute-module.html)
3. Set an operator account and SSH public key through the image's supported Imager/cloud-init provisioning. Use Ethernet and a DHCP reservation for the initial setup. Check the address with `ip -br addr` or the DHCP server, verify the SSH host key through a trusted console, then connect with `ssh OPERATOR@CM5_ADDRESS`. Confirm key login before disabling password authentication. Updates need internet access. Boot and passive operation use the local build and PX4 link.

On the host, verify the results before installing:

```bash
cat /etc/os-release
uname -m; uname -r; dpkg --print-architecture
tr -d '\0' </proc/device-tree/model; echo
lsblk -o NAME,SIZE,MODEL,TRAN,MOUNTPOINTS
ip -br addr
df -h /opt; free -h
```

For the 24.04 host, set up LXD on the CM5 and create an unprivileged system container. Use at least 32 GB of usable storage for multiple builds and logs, and preferably 4 GB RAM or more. Run the remaining deployment commands inside Jammy. Record the image fingerprint for fleet replication.

```bash
sudo snap install lxd --channel=5.21/stable
sudo lxd init
sudo lxc launch ubuntu:22.04 drone-humble
sudo lxc config set drone-humble boot.autostart true
sudo lxc exec drone-humble -- sudo -iu ubuntu
cat /etc/os-release
uname -m; dpkg --print-architecture
```

The container uses the host kernel, so confirm the device works on the host before passing it through. For UART or USB serial, add only the selected character device. Set `DEVICE_ON_HOST` to the actual stable path on the CM5. Run this on the host and configure `PX4_SERIAL_DEVICE=/dev/px4` inside the container. Verify device permissions and reconnect behavior on your carrier. [LXD guest compatibility](https://documentation.ubuntu.com/lxd/latest/guest-os-compatibility/), [LXD character devices](https://canonical.com/lxd/docs/stable-5.21/reference/devices_unix_char/)

```bash
GID=$(sudo lxc exec drone-humble -- getent group dialout | cut -d: -f3)
sudo lxc config device add drone-humble px4 unix-char source="$DEVICE_ON_HOST" \
    path=/dev/px4 gid="$GID" mode=0660 required=false
```

## Install, build and select a release

On Jammy, enable Ubuntu Universe, then download the official Jammy `ros2-apt-source` release asset to `~/ros2-apt-source.deb` (record the asset version and checksum). The installer accepts this local package on first run or reuses an installed source package. It installs ROS Base and development packages, initializes rosdep if needed, compiles the Agent at v2.4.2 (`57d086216d01ec43121845d385894a25987f8a2c`) against Humble's Fast DDS 2.6/Fast CDR 1.x, and installs the service files. Configuration already on disk is retained. Apt dependencies follow Jammy/Humble repositories; save `dpkg-query -W` output if recording an exact machine baseline. [ROS apt source release assets](https://github.com/ros-infrastructure/ros-apt-source/releases), [PX4 XRCE version matrix](https://docs.px4.io/main/en/middleware/uxrce_dds#version-selection)

```bash
sudo apt-get update
sudo apt-get install -y git software-properties-common
sudo add-apt-repository -y universe
git clone --branch codex/phase7-cm5-deployment https://github.com/hoangv77/drone_project.git
cd drone_project
bash deployment/cm5/install.sh "$HOME/ros2-apt-source.deb"
```

Select `PX4_COMMIT` as the full firmware source SHA from the controller console's `ver all` and the exact flashed build's records. For a custom build, its committed source must include local schema and `dds_topics.yaml` changes. Record the board target, firmware artifact checksum and DDS client configuration. The client must support XRCE v2. Clone the firmware source (or supply the exact matching vendor Git repository) and fetch the commit. For **isolated Phase 6 SITL only**, use `97a3c7f01793e2b0b9e5a30425b8e88e69741f27`.

```bash
git clone --no-checkout https://github.com/PX4/PX4-Autopilot.git "$HOME/px4-firmware-source"
PX4_SOURCE="$HOME/px4-firmware-source"
# Set PX4_COMMIT to the audited 40-character commit, then run:
git -C "$PX4_SOURCE" fetch origin "$PX4_COMMIT"
bash deployment/cm5/build.sh "$PX4_SOURCE" "$PX4_COMMIT"
# Set RELEASE to the full "Verified release" path printed by build.sh.
bash deployment/cm5/verify.sh build "$RELEASE"
bash deployment/cm5/update.sh --activate "$RELEASE"
```

The build uses a clean Git snapshot and a new directory under `/opt/drone/releases`. The schema tool extracts the exact firmware Git objects, stages `px4_msgs` using the Phase 6 tool, checks telemetry topic types and version suffixes against the node's contract, and records the SHA, DDS configuration digest, message versions and schema hashes in `schema.json`. Compilation catches unsupported fields and constants. The installed schema files are checked against the manifest. This establishes source/overlay consistency. Match the connected device to the firmware artifact separately. The build uses sequential colcon and single-job compilation. The installer leaves services disabled on a fresh installation.

## Connect PX4 and start monitoring

Use a propeller-free bench setup for all initial physical tests. For UART, use the carrier and flight controller manuals to confirm pinout, electrical levels, common reference and compatible signalling. Cross TX/RX for a compatible TTL UART. Verify power wiring separately and disable any conflicting console or getty on that port. Prefer a stable `/dev/serial/by-id/` or board-specific udev link where available. For USB, confirm the selected PX4 firmware actually exposes an XRCE-capable CDC serial interface. The USB connector may instead expose MAVLink or a bootloader. On Linux, use `ls -l /dev/serial/by-id`, `ls -l /dev/ttyUSB* /dev/ttyACM*` and `udevadm info --name="$DEVICE"` to identify and inspect the device.

Use QGroundControl/PX4 console and the exact firmware's parameter docs to set `UXRCE_DDS_CFG` to the selected port, enable the XRCE client and match the port's baud rate, such as `SER_TEL2_BAUD` for a supported TELEM2 port. Reassign any existing service that owns that same port. Set `UXRCE_DDS_DOM_ID` to the configured ROS domain and `UXRCE_DDS_PTCFG` to localhost-only (value 1 on the Phase 6 PX4 line). Use unprefixed topics and reboot PX4 as required. Check `uxrce_dds_client status`. For UDP SITL, set the client target to the Agent's IP/port and use an isolated network or loopback. For a container, run SITL inside the container for loopback, or route UDP explicitly over a dedicated test network. XRCE UDP listens on the configured port and needs network isolation. [PX4 client configuration](https://docs.px4.io/main/en/middleware/uxrce_dds#starting-the-client)

```bash
sudoedit /etc/drone/drone.env
# Set PX4_TRANSPORT=serial (UART), usb (USB serial), or udp (SITL).
# Set PX4_SERIAL_DEVICE and PX4_BAUD_RATE for serial/USB.
# Set PX4_AGENT_UDP_PORT for UDP, and match ROS_DOMAIN_ID on PX4.
sudo systemctl start drone-agent
sudo systemctl start drone-stack
sudo systemctl status drone-agent drone-stack
sudo journalctl -u drone-agent -u drone-stack -f
```

After successful checks and when ready for boot startup:

```bash
sudo systemctl enable drone-agent
sudo systemctl enable drone-stack
```

For shutdown:

```bash
sudo systemctl stop drone-stack drone-agent
# To remove automatic boot startup:
sudo systemctl disable drone-stack drone-agent
```

The stack can monitor a disconnected controller. An unavailable serial device or failed Agent is logged and retried at five-second intervals. A required ROS process exit shuts the launch down so systemd can restart the complete stack. To tune monitor timeouts or safety parameters, copy the package YAML to `/etc/drone`, review the policy implications and pass `safety_config:=/etc/drone/safety.yaml` or `mission_config:=/etc/drone/mission.yaml` through `run.sh stack` in a systemd `ExecStart` override. The launch sets read-only and monitoring-only parameters explicitly.

## Verification and diagnostics

```bash
bash deployment/cm5/verify.sh passive   # disconnected launch and interlocks
bash deployment/cm5/verify.sh sitl      # isolated UDP SITL
bash deployment/cm5/verify.sh hardware  # propeller-free serial or USB bench
sudo journalctl -u drone-stack -u drone-agent -b --no-pager
```

Live checks require a dedicated domain and verify the state, safety and mission streams. SITL/bench modes also require a running Agent and fresh PX4 data, with observed landed/disarmed state. A fresh safety report can legitimately say BLOCKED. Review the safety reasons before any later flight work.

For ROS CLI diagnostics in the same machine or system container:

```bash
source /opt/ros/humble/setup.bash
source /opt/drone/state/current/install/local_setup.bash
set -a; source /etc/drone/drone.env; set +a
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/etc/drone/fastdds.xml
ros2 topic list --no-daemon
ros2 topic echo /fmu/out/failsafe_flags --qos-reliability best_effort --once
ros2 topic echo /drone/state --once
ros2 topic echo /drone/safety_status --once
ros2 topic echo /drone/mission_state --once
```

If PX4 topics are absent, check the device, service-account access, baud, port ownership, client status, transport and domain. If topics appear without data or freshness, check best-effort QoS, schema hashes, topic suffixes and PX4 publication rates. A single fresh topic is insufficient. Repeat passive checks after disconnect, reconnect and reboot. For container USB reconnect problems inspect both host and container device nodes and the Agent journal.

## Remote updates and rollback

SSH to the CM5 and enter Jammy if using LXD. Use the same operator account and an unchanged repository checkout. Select an explicit reviewed commit or tag on GitHub and the verified firmware revision. The updater builds from a detached worktree, verifies the candidate before stopping services, then switches the release symlink. It restores the prior release if activation or the passive startup check fails. It preserves service enablement and uses `/opt/drone/state/deployments.log` for timestamped commit history. Unit and configuration changes need a separately reviewed install step. Previous builds are retained for rollback and disk use should be monitored.

```bash
ssh OPERATOR@CM5_ADDRESS
# For LXD: sudo lxc exec drone-humble -- sudo -iu ubuntu
cd ~/drone_project
bash deployment/cm5/update.sh "$APPROVED_GIT_REF" "$PX4_SOURCE" "$PX4_COMMIT"
bash deployment/cm5/verify.sh passive
# To roll back, select the previous verified path from deployments.log:
bash deployment/cm5/update.sh --activate "$PREVIOUS_RELEASE"
```

## Tests and physical-flight prerequisites

Run deployment tests inside the supported userspace. Use an isolated host/domain for the ROS test, which calls the onboard services to verify rejection. Use the release build, test and install directories together.

```bash
python3 -m unittest discover -s deployment/cm5/tests -v
source /opt/ros/humble/setup.bash
source "$RELEASE/install/local_setup.bash"
CM5_TEST_ROS=1 python3 -m unittest discover -s deployment/cm5/tests -v
CM5_TEST_SYSTEMD=1 python3 -m unittest discover -s deployment/cm5/tests -p test_systemd.py -v
(cd "$RELEASE" && colcon test --merge-install --packages-select px4_interface_node safety_monitor state_machine_manager)
(cd "$RELEASE" && colcon test-result --verbose)
```

Before any physical flight, record the actual carrier, boot image/kernel and firmware revision. Validate power, thermal headroom, storage, ports, cold boot and recovery. Confirm telemetry freshness and permissions, unplug/reconnect behavior, boot restart, update rollback, and PX4 failsafes with propellers removed. Independently verify propulsion, mass/CG, controller orientation, calibration, and operator link and recovery procedures. Flight authority requires a separate controlled test plan and approval. Keep hardware Throw Launch disabled until its orientation failure and flight safety prerequisites have been addressed and independently validated.
