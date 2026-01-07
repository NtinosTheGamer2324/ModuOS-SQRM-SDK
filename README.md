# ModuOS Driver SDK (v0.5.5 Base)

This repository serves as the standalone Software Development Kit for ModuOS driver development. It includes the updated resource management headers and source code for experimental display drivers.

### ⚠️ Compatibility & ABI Notice
**Breaking Changes in v0.5.6:**
The GPU drivers included in this SDK (QXL, VMSVGA) are architected for the **ModuOS 0.5.6** branch, which is currently in private development.

These drivers utilize the revised **SQRM ((SQRELFM) Squirrel Executable and Linkable Format Module)** architecture. Specifically, 0.5.6 introduces two new resource types:
* **Type 5:** GPU / Display Resources
* **Type 6:** Network Stack / Interface Resources

**Version Interoperability:**
While these drivers are incompatible with the **v0.5.5** public release, the kernel will not crash upon attempted loading. Due to the SQRM ABI design, a version mismatch (v1.1 vs v1.0) will result in an "unsupported resource type" error rather than a kernel panic. However, the drivers will remain non-functional until the 0.5.6 kernel-base is deployed.

---

### Included Components

#### Experimental GPU Drivers (v0.5.6)
* **QXL Driver:** Optimized for QEMU/KVM; implements command ring processing and primary surface management.
* **VMSVGA Driver:** Designed for VMware/VirtualBox; includes hardware cursor support and SVGA register control.

#### SDK Base
* **SQRM Headers:** Updated definitions including the new Type 5 and Type 6 assignments.
* **PCI Abstraction:** Revised discovery logic for identifying accelerated graphics hardware.
* **MMIO/IO Helpers:** Standardized interfaces for hardware-level communication.

### Implementation Status
The SDK is built upon the 0.5.5 environment, but the driver logic itself relies on the 0.5.6 kernel's ability to dispatch Type 5/6 requests. This repository is provided for developers to study the new driver model and SQRELFM integration ahead of the full 0.5.6 release.

### Main Project
The stable ModuOS kernel and previous public releases are available at:

**[https://github.com/NtinosTheGamer2324/ModuOS](https://github.com/NtinosTheGamer2324/ModuOS)**