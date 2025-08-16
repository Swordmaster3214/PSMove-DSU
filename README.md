# PSMove-DSU
A Cross Platform DSU/Cemuhook Server using psmoveapi for Dolphin Emulator to understand the Playstation Move controller's accurate sixaxis orientation stuff.
## Usage instructions
If your controller is already paired with your PC, just run the server. If not, connect your PS Move controller via USB and let the automatic pairing utility handle it for you, remember to disconnect the controller and power it on within the 12 second window. Note that it may require elevated privileges. Connect your DSU server (localhost, port 26760) and map your buttons in Dolphin or some other DSU-supported emulator as you please. Note that you do not need psmoveapi installed in order to run the binary. Also, only one controller at a time for now.
### Button Mappings

Button | Mapping
--- | ---
X | Cross
O | Circle
□ | Square
∆ | Triangle
Move | R1
SELECT | Share
START | Options
T | R2
PS | PS
## Compiling instructions
### Linux (Arch, Debian, Ubuntu, etc.)

**Dependencies:**
- g++ (C++17 or newer)
- psmoveapi (on Arch: psmoveapi-git from the AUR, or build from source, or use the release)

**Build:**
```
g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove -lpsmoveapi -lpthread
```

**Run:**
```
./dsu_server_psmove
```

---

### Windows

#### MinGW (MSYS2)

**Dependencies:**
```
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-psmoveapi make
```

**Build:**
```
g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove.exe -lpsmoveapi -lws2_32 -lwinmm
```

**Run:**
```
./dsu_server_psmove.exe
```

#### MSVC (Visual Studio)

1. Create a new **Console App** project.  
2. Add `dsu_server_psmove.cpp` to the project.  
3. Set the C++ language standard to **C++17**.  
4. Link against `psmoveapi.lib` (build from source if not provided).  
5. Ensure `psmoveapi.dll` is available in your PATH or in the same folder as the `.exe`.  

Build and run inside Visual Studio.

---

### macOS

**Dependencies:**
- Xcode Command Line Tools:  
  ```
  xcode-select --install
  ```
- [Homebrew](https://brew.sh/), then:  
  ```
  brew install psmoveapi
  ```

**Build:**
```
clang++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove -lpsmoveapi -lpthread
```

**Run:**
```
./dsu_server_psmove
```

## Licensing
License details for this project are in the LICENSE file.

The PS Move API library is licensed under the terms of the license below.
However, some optional third party libraries might have a different license.
Be sure to read the README file for details on third party licenses.

====

Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
