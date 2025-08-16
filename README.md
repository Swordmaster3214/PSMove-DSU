# PSMove-DSU
A Cross Platform DSU Server using psmoveapi for Dolphin Emulator to understand the Playstation Move controller's accurate sixaxis orientation stuff.
## Usage instructions
Make sure you have already paired your PS Move controller with your system and have it turned on. Then run the DSU server, and you will be good to go. Connect your DSU server and ap your buttons in Dolphin or some other DSU-supported emulator as you please. Note that you do not need psmoveapi installed in order to run the binary, however, you need to have had it installed at one point to pair the controller. I plan to change this in the future, with the pairing process built in.
## Compilation instructions
Ensure that you have https://github.com/thp/psmoveapi installed, as that is a required dependency for this project.
### Linux/macOS
```
g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove -pthread $(pkg-config --cflags --libs libpsmoveapi)
```
### Windows (MinGW)
```
g++ -O2 -std=c++17 dsu_server_psmove.cpp -o dsu_server_psmove.exe -lpsmoveapi -lws2_32 -pthread
```
### Windows (MSVC)
```
cl /O2 /std:c++17 dsu_server_psmove.cpp /link psmoveapi.lib ws2_32.lib
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
