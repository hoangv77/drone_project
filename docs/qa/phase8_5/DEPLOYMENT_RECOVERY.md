# CM5 activation and recovery

QA-09. The previous updater relied on exit traps after changing current. The new journal fsyncs durable activation phases, atomically switches current, and records verification only after both passive services are running and the passive health check passes. A verifier process cannot inherit the updater lock. A stable installed service entry point supervises the provisional release and exits if live verification loses its updater. On service restart the installed boot helper reconciles an incomplete activation to the last runtime-verified release. Corruption, an inconsistent symlink or an unavailable previous release blocks service startup. Initial activation with no prior verified release rolls back to no selection.

Review and install the new systemd unit files and the root-owned recovery helper through install.sh before activation. Existing units are deliberately left for operator review on mismatch. A Phase 8 legacy symlink alone does not establish runtime verification. Explicitly activate its built release with the new updater if migration is required. Confirm configuration and target release before the first passive start. The service helper assumes a trusted administrator and deployment operator, local Linux filesystem semantics, and systemd restart behavior. Fault fixtures cover transaction boundaries and storage failure. Validate an actual ARM64 power cut, boot reconciliation and services on the target storage before operational approval. [Journal](../../../deployment/cm5/activation.py) · [Updater](../../../deployment/cm5/update.sh)

For a reviewed upgrade in a passive bench maintenance window, stop the two services, install the new unit files, then rerun the installer and activate the intended built release. Activation verifies both services and restores their earlier running/stopped status. Enable automatic startup only after target validation.

```bash
sudo systemctl stop drone-stack drone-agent
sudo install -m 0644 deployment/cm5/systemd/drone-{agent,stack}.service /etc/systemd/system/
sudo systemctl daemon-reload
bash deployment/cm5/install.sh
bash deployment/cm5/update.sh --activate "$RELEASE"
```
