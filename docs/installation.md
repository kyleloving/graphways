# Installation

## Python

### From PyPI

```bash
pip install graphways
```

Wheels are provided for CPython 3.8 and later on Linux (x86-64 and ARM64),
macOS (Apple silicon and Intel) and Windows (x86-64): one wheel per platform
covers every Python version. On other platforms pip builds from the source
distribution, which needs [Rust](https://rustup.rs/).

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

The default `network` feature downloads data from Overpass and Nominatim. If
you only load local PBF or XML files, leave it out to skip the HTTP stack:

```toml
[dependencies]
graphways = { version = "0.5.0", default-features = false }
```

## Dependencies

graphways has no required Python dependencies -- all heavy lifting is in Rust.

Optional Python packages used in the examples:

```bash
pip install folium branca
```
