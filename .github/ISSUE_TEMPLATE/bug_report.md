---
name: Bug report
about: Create a report to help me improve PSMove-DSU
title: "[BUG]"
labels: bug
assignees: ''

---

## Bug Description
**A clear and concise description of what the bug is.**

## Steps to Reproduce
1. Go to '...'
2. Run command '...'
3. Connect controller '...'
4. See error

## Expected Behavior
**What you expected to happen.**

## Actual Behavior
**What actually happened instead.**

## System Information
**Please complete the following information:**

- **OS**: [e.g. Windows 11, Ubuntu 22.04, macOS 13.0]
- **Architecture**: [e.g. x64, ARM64]
- **PSMove-DSU Version**: [e.g. v2.1.0]
- **Controller Model**: [e.g. CECH-ZCM1U, CECH-ZCM2U]
- **Connection Type**: [e.g. Bluetooth, USB]

## Controller Information
**Run with `--verbose` and provide controller details:**

```
Paste controller serial, connection type, and any relevant controller logs here
```

## Performance Information
**If related to latency/performance:**

- **Latency warnings in logs**: [Yes/No - paste any latency warnings]
- **CPU usage during issue**: [e.g. 15%, unknown]
- **Other running applications**: [e.g. games, streaming software]

## Verbose Logs
**Run `./dsu_server_psmove --verbose` and paste relevant log output:**

```
Paste verbose logs here, especially around the time the issue occurs.
Include at least 10-20 lines before and after the problem.
```

## Troubleshooting Attempted
**What have you already tried?**

- [ ] Restarted the application
- [ ] Re-paired the controller (`--pair` mode)
- [ ] Checked Bluetooth connection
- [ ] Tested with different USB cable (if applicable)
- [ ] Checked port 26760 availability
- [ ] Tested with other DSU clients (Dolphin, Cemu, etc.)
- [ ] Tested on different computer/OS

## DSU Client Information
**If the issue involves DSU client connectivity:**

- **Client Application**: [e.g. Dolphin 5.0, Cemu 1.27.1, Custom]
- **Client OS**: [if different from server OS]
- **Network Setup**: [same computer, different computer, IP addresses if relevant]

## 📎 Additional Context
**Add any other context about the problem here.**

- Screenshots (if applicable)
- Configuration files (if modified)
- Any recent system changes
- Other controllers or software that might interfere

## Urgency Level
- [ ] **Critical** - Application crashes or completely unusable
- [ ] **High** - Major functionality broken, workaround exists
- [ ] **Medium** - Minor functionality issues or performance problems  
- [ ] **Low** - Small inconveniences or feature improvements

---

**Note**: For performance issues, please include `--verbose` logs as they contain critical latency measurements. Issues without sufficient information may be closed and tagged as "needs more info".
