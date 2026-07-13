# 23-CompletionTree

Source code for this paper:
https://www.user.tu-berlin.de/mtoussai/24-CompletionTrees/

Try `make` src/

Dependencies need to be installed, e.g. as for the https://github.com/MarcToussaint/robotic package. I think this might be sufficient:

```
sudo apt install --yes \
  g++ clang make gnupg cmake git wget libstdc++-14-dev \
  liblapack-dev libf2c2-dev libqhull-dev libeigen3-dev \
  libjsoncpp-dev libyaml-cpp-dev libhdf5-dev libpoco-dev libboost-system-dev portaudio19-dev libusb-1.0-0-dev \
  libx11-dev libglu1-mesa-dev libglfw3-dev libglew-dev freeglut3-dev libpng-dev libassimp-dev
  
wget https://github.com/MarcToussaint/rai/raw/refs/heads/marc/_make/install.sh; chmod a+x install.sh
./install.sh libccd
./install.sh fcl
./install.sh libann
```

## Tested build on Fedora

The `apt` list above is Debian/Ubuntu-specific; here's the from-scratch sequence actually
tested on Fedora (43), including three gotchas the `apt` instructions above don't need to deal
with. Assumes only `git`, a C++ toolchain, `cmake`, and `make` are present (i.e. nothing else
in this repo has been touched yet).

**1. Clone with submodules** (this repo doesn't vendor `rai` or the benchmark problems):
```bash
git clone --recurse-submodules <this-repo-url> 23-CompletionTree
cd 23-CompletionTree
# if you cloned without --recurse-submodules:
git submodule update --init --recursive
```

**2. Install system packages.** Fedora's package names differ from Debian's; this covers
everything the `apt` list above does, one-for-one (if you have `sudo`):
```bash
sudo dnf install -y \
  gcc-c++ clang make cmake git wget \
  lapack-devel qhull-devel eigen3-devel \
  jsoncpp-devel yaml-cpp-devel hdf5-devel poco-devel poco-foundation poco-net poco-util \
  poco-xml poco-json poco-crypto poco-netssl boost-devel portaudio-devel libusb1-devel \
  libX11-devel mesa-libGLU-devel glfw-devel glew-devel freeglut-devel libpng-devel \
  assimp-devel libccd-devel fcl-devel ann-devel
```
(`libccd`/`fcl`/`ann` -- the three packages the upstream instructions build via `rai`'s own
`install.sh` -- are ordinary Fedora packages, `libccd-devel`/`fcl-devel`/`ann-devel`; no need
for that script here.)

**No `sudo` available?** Everything above can still be obtained without root: `dnf download
--resolve --arch=x86_64 <pkg>...` works unprivileged and fetches the RPM plus its
dependencies; extract each with `rpm2cpio <rpm>.rpm | cpio -idm` into a scratch prefix, then
copy/symlink the resulting `usr/include/*` and `usr/lib64/*` into `~/.local/{include,lib}` --
`rai`'s own `makeutils/generic.mk` already searches `~/.local/{include,lib}` by default, so no
`rai`-side changes are needed either way. Watch for two follow-up issues this route (but not
the `sudo` route) tends to hit: (a) dangling `libfoo.so -> libfoo.so.N` symlinks, when the
*runtime* package (as opposed to `-devel`) was already installed system-wide under a path the
extraction didn't touch -- repoint them at the real `.so.N` under `/usr/lib64`; (b)
`pkg-config`'s copied `.pc` files keep their original `/usr` `prefix=`/`libdir=`/`includedir=`
values, which resolve to paths nothing was actually copied to -- rewrite those four lines to
point at `~/.local` in every `.pc` file you copy in.

**3. Two Fedora-specific gotchas, regardless of which install route you used above:**

- **No package on Fedora ships `f2c.h`** (checked: not even a `f2c-devel` package exists) --
  `rai/Core/array.cpp` unconditionally `#include`s it before branching into either the classic
  CLAPACK path or the modern LAPACKE path (Fedora's `lapack-devel` only has the latter).
  `.deps/include/f2c.h` in this repo is a minimal, ABI-compatible shim (just the typedefs
  `lapack/clapack.h` needs -- `integer`, `doublereal`, `VOID`, `L_fp`, ...; ABI-compatible with
  the real `liblapack.so.3` since that's reference LAPACK with standard Fortran
  underscore-suffixed symbol names). `.deps/env.sh` (source it before building, see below) puts
  it on `CPATH` automatically.
- **Fedora's `jsoncpp-devel` installs headers one directory shallower than the code expects**
  (`<prefix>/include/json/json.h`, but `rai`'s code does `#include <jsoncpp/json/json.h>`, the
  Debian/Ubuntu layout). `.deps/env.sh` self-heals this too -- it creates a
  `~/.local/include/jsoncpp/json -> .../json` symlink pointing at whichever `json/json.h` it
  finds (system or user-local), the first time you source it.

```bash
source .deps/env.sh   # every gotcha above, handled; safe to source repeatedly
```

**4. Disable the two dependencies not needed here** (both are on by default in individual
modules' Makefiles and only get overridden by the top-level `config.mk`, which is missing
these two lines by default -- Python bindings and Graphviz plotting aren't needed for anything
in this repo's `src/` or `src_hopscotch/`):
```bash
cat >> config.mk << 'EOF'
PYBIND = 0
GRAPHVIZ = 0
EOF
```
(Skipping this: `PYBIND` wants `pip install pybind11` wired up in a very particular way
unrelated to anything built here; `GRAPHVIZ` wants `libcgraph`/`libgvc`, not packaged the same
way on Fedora.)

**5. Build:**
```bash
export LD_LIBRARY_PATH="$(pwd)/rai/lib:$LD_LIBRARY_PATH"
cd rai && make src && cd ..
cd src && make   # -> src/x.exe, the paper's own experiments
```

**6. Run the paper's own experiments.** `src/rai.cfg`'s `problem`/`problems` entries need to be
absolute paths into this checkout's own `lgp-benchmarks/` (rai's config-file parser mis-parses
list entries starting with `../`, so plain relative paths don't work here). Rather than hardcode
any one machine's path, `rai.cfg` is generated from the checked-in `src/rai.cfg.in` template by
`./generate-rai-cfg.sh`, which substitutes this checkout's own absolute path for the
`@REPO_ROOT@` placeholder -- `cd src && make` already runs this for you (it's a build
prerequisite), but you can also run it directly, including to re-point `rai.cfg` after moving or
re-cloning the checkout:
```bash
./generate-rai-cfg.sh   # only needed directly if you're not about to `make` in src/ anyway
cd src && ./x.exe -LGP/verbose 0   # runs every problem in rai.cfg's `problems` list, K=10 restarts each
```
One of the default `problems` list entries, `robot-pnp/pr2-onTray.lgp`, references PR2 mesh
files at a path layout (`pr2/meshes/base_v0/*.ply`) the current
[rai-robotModels](https://github.com/MarcToussaint/rai-robotModels) repo no longer has (meshes
moved to `.h5`) -- drop it from `rai.cfg`'s `problems` list unless you also pin an old
`rai-robotModels` commit contemporaneous with `lgp-benchmarks`.

## Hopscotch benchmark port (`src_hopscotch/`)

`src_hopscotch/` ports [hopscotch](https://github.com/KavrakiLab/hopscotch)'s own
pick-and-place/stacking/coffee/mobile TAMP benchmark problems into this repo's own completion-tree
search (`rai::AStar` over `rai::ComputeNode`), calling into hopscotch's real Rust geometry (IK,
motion planning, grasp sampling, collision) through a small C ABI crate. See
`hopscotch/examples/pddlstream/README.md`'s "Third system: 23-CompletionTree" section for the
full setup, build, and benchmark-reproduction instructions (it assumes steps 1-5 above are
already done).
