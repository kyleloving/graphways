# Installation

## Python

### From PyPI

```bash
pip install graphways
```

Wheels are provided for Python 3.8+ on Linux, macOS, and Windows (x86-64).

### From source

Requires [Rust](https://rustup.rs/) and [maturin](https://www.maturin.rs/).

```bash
git clone https://github.com/kyleloving/graphways.git
cd graphways
pip install maturin
maturin develop --release
```

`maturin develop` compiles the Rust extension and installs it into the current Python environment in one step. The `--release` flag enables compiler optimizations -- omit it only for debug builds.

## Rust

Add to `Cargo.toml`:

```toml
[dependencies]
graphways = "0.5.0"
```

> **Note:** The crate is published as `graphways`; the library module is `graphways` (matching the Python package name).

## Dependencies

graphways has no required Python dependencies -- all heavy lifting is in Rust.

Optional Python packages used in the examples:

```bash
pip install folium branca
```
