# K-OS Embassy — Developer Setup

> Tested on macOS + `rustup` tool-chain 1.85

---

## 1. Install Rust (rustup)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

# Choose Default Installation

```bash
git clone git@github.com:kscalelabs/kos-embassy.git
cd kos-embassy
rustup target add thumbv6m-none-eabi

curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh


cargo build --release
```
