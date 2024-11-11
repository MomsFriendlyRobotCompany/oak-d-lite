# Conan Notes

## Setup

```bash
python3 -m venv venvs/conan
changevenv conan
pip install -U setuptools pip wheel conan
```

```bash
$ conan profile detect
detect_api: Found apple-clang 14.0
detect_api: apple-clang>=13, using the major as version

Detected profile:
[settings]
arch=x86_64
build_type=Release
compiler=apple-clang
compiler.cppstd=gnu17
compiler.libcxx=libc++
compiler.version=14
os=Macos

WARN: This profile is a guess of your environment, please check it.
WARN: Defaulted to cppstd='gnu17' for apple-clang.
WARN: The output of this command is not guaranteed to be stable and can change in future Conan versions.
WARN: Use your own profile files for stability.
Saving detected profile to /Users/kevin/.conan2/profiles/default
```

## Setup Repo

Add `conanfile.txt` to same directory as `CMakeLists.txt` with needed libraries:

```ini
[requires]
mcap/1.4.0
zlib/1.2.11

[generators]
CMakeDeps
CMakeToolchain

[layout]
cmake_layout
```

`[layout] cmake_layout` automatically puts all of the conan garbage files in build/Release/generators and keeps things nice and clean

Run from source directory where conanfile.txt and CMakeLists.txt is. If libraries
are new and not installed yet, the `--build=missing` will download and install
to `~/.conan2`.

```bash
conan install . --build=missing
```