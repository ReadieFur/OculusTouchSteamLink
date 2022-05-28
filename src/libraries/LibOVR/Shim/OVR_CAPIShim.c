/************************************************************************************

Filename    :   OVR_CAPIShim.c
Content     :   CAPI DLL user library
Created     :   November 20, 2014
Copyright   :   Copyright (c) Facebook Technologies, LLC and its affiliates. All rights reserved.

Licensed under the Oculus Master SDK License Version 1.0 (the "License");
you may not use the Oculus VR Rift SDK except in compliance with the License,
which is provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

You may obtain a copy of the License at

https://developer.oculus.com/licenses/oculusmastersdk-1.0

Unless required by applicable law or agreed to in writing, the Oculus VR SDK
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

************************************************************************************/

#include <assert.h>
#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#include "OVR_CAPI.h"
#include "OVR_CAPI_Prototypes.h"
#include "OVR_ErrorCode.h"
#include "OVR_Version.h"

#if defined(_WIN32)
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif
#include <Windows.h>
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#include "OVR_CAPI_D3D.h"
#pragma comment(lib, "advapi32.lib")
#else
#if defined(__APPLE__)
#include <libgen.h>
#include <mach-o/dyld.h>
#include <pwd.h>
#include <sys/syslimits.h>
#include <unistd.h>
#endif
#include <dlfcn.h>
#include <sys/stat.h>
#include <unistd.h>
#endif
#include "OVR_CAPI_GL.h"
#include "OVR_CAPI_Vk.h"


// clang-format off
static const uint8_t OculusSDKUniqueIdentifier[] = {
  0x9E, 0xB2, 0x0B, 0x1A, 0xB7, 0x97, 0x09, 0x20, 0xE0, 0xFB, 0x83, 0xED, 0xF8, 0x33, 0x5A, 0xEB,
  0x80, 0x4D, 0x8E, 0x92, 0x20, 0x69, 0x13, 0x56, 0xB4, 0xBB, 0xC4, 0x85, 0xA7, 0x9E, 0xA4, 0xFE,
  OVR_MAJOR_VERSION, OVR_MINOR_VERSION, OVR_PATCH_VERSION
};

// clang-format on

static const uint8_t OculusSDKUniqueIdentifierXORResult = 0xcb;

// -----------------------------------------------------------------------------------
// ***** OVR_BUILD_DEBUG
//
// Defines OVR_BUILD_DEBUG when the compiler default debug preprocessor is set.
//
// If you want to control the behavior of these flags, then explicitly define
// either -DOVR_BUILD_RELEASE or -DOVR_BUILD_DEBUG in the compiler arguments.

#if !defined(OVR_BUILD_DEBUG) && !defined(OVR_BUILD_RELEASE)
#if defined(_MSC_VER)
#if defined(_DEBUG)
#define OVR_BUILD_DEBUG
#endif
#else
#if defined(DEBUG)
#define OVR_BUILD_DEBUG
#endif
#endif
#endif

//-----------------------------------------------------------------------------------
// ***** FilePathCharType, ModuleHandleType, ModuleFunctionType
//
#if defined(_WIN32) // We need to use wchar_t on Microsoft platforms, as that's the native file
// system character type.
#define FilePathCharType \
  wchar_t // #define instead of typedef because debuggers (VC++, XCode) don't recognize typedef'd
// types as a string type.
typedef HMODULE ModuleHandleType;
typedef FARPROC ModuleFunctionType;
#else
#define FilePathCharType char
typedef void* ModuleHandleType;
typedef void* ModuleFunctionType;
#endif

#define ModuleHandleTypeNull ((ModuleHandleType)NULL)
#define ModuleFunctionTypeNull ((ModuleFunctionType)NULL)

//-----------------------------------------------------------------------------------
// ***** OVR_MAX_PATH
//
#if !defined(OVR_MAX_PATH)
#if defined(_WIN32)
#define OVR_MAX_PATH _MAX_PATH
#elif defined(__APPLE__)
#define OVR_MAX_PATH PATH_MAX
#else
#define OVR_MAX_PATH 1024
#endif
#endif

#if !defined(OVR_DLSYM)
#if defined(_WIN32)
#define OVR_DLSYM(dlImage, name) GetProcAddress(dlImage, name)
#else
#define OVR_DLSYM(dlImage, name) dlsym(dlImage, name)
#endif
#endif

static size_t OVR_strlcpy(char* dest, const char* src, size_t destsize) {
  const char* s = src;
  size_t n = destsize;

  if (n && --n) {
    do {
      if ((*dest++ = *s++) == 0)
        break;
    } while (--n);
  }

  if (!n) {
    if (destsize)
      *dest = 0;
    while (*s++) {
    }
  }

  return (size_t)((s - src) - 1);
}

static size_t OVR_strlcat(char* dest, const char* src, size_t destsize) {
  const size_t d = destsize ? strlen(dest) : 0;
  const size_t s = strlen(src);
  const size_t t = s + d;

  if (t < destsize)
    memcpy(dest + d, src, (s + 1) * sizeof(*src));
  else {
    if (destsize) {
      memcpy(dest + d, src, ((destsize - d) - 1) * sizeof(*src));
      dest[destsize - 1] = 0;
    }
  }

  return t;
}

#if defined(_WIN32)
static size_t OVR_wstrlcat(wchar_t* dest, const wchar_t* src, size_t destsize) {
  const size_t d = destsize ? wcslen(dest) : 0;
  const size_t s = wcslen(src);
  const size_t t = s + d;

  if (t < destsize)
    memcpy(dest + d, src, (s + 1) * sizeof(*src));
  else {
    if (destsize) {
      memcpy(dest + d, src, ((destsize - d) - 1) * sizeof(*src));
      dest[destsize - 1] = 0;
    }
  }

  return t;
}
#endif // defined(_WIN32)

#if defined(_WIN32)
static ovrBool OVR_GetOculusBaseDirectory(
    FilePathCharType* directoryPath,
    size_t directoryPathCapacity) {
  DWORD length = (DWORD)directoryPathCapacity;
  HKEY key = 0;
  // Query the 32bit key even if 64bit build
  LONG result = RegOpenKeyExW(
      HKEY_LOCAL_MACHINE, L"Software\\Oculus VR, LLC\\Oculus", 0, KEY_READ | KEY_WOW64_32KEY, &key);
  if (result != ERROR_SUCCESS) {
    return ovrFalse;
  }

  result = RegGetValueW(
      key, NULL, L"Base", RRF_RT_REG_SZ | RRF_ZEROONFAILURE, NULL, (LPBYTE)directoryPath, &length);
  RegCloseKey(key);

  return result == ERROR_SUCCESS ? ovrTrue : ovrFalse;
}

static ovrBool IsHighIntegrityLevel() {
  // Allocate a fixed size buffer for the maximum possible size:
  // SID_AND_ATTRIBUTES can be the size of a SID + size of DWORD (for attributes).
  uint8_t mandatoryLabelBuffer[SECURITY_MAX_SID_SIZE + sizeof(DWORD)];
  DWORD bufferSize;
  HANDLE processToken;
  DWORD integrityLevel = SECURITY_MANDATORY_HIGH_RID;

  if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY | TOKEN_QUERY_SOURCE, &processToken) == 0) {
    return ovrFalse;
  }

  if (GetTokenInformation(
          processToken,
          TokenIntegrityLevel,
          mandatoryLabelBuffer,
          sizeof(mandatoryLabelBuffer),
          &bufferSize) != 0) {
    TOKEN_MANDATORY_LABEL* mandatoryLabel = (TOKEN_MANDATORY_LABEL*)mandatoryLabelBuffer;
    const DWORD subAuthorityCount = *GetSidSubAuthorityCount(mandatoryLabel->Label.Sid);
    integrityLevel = *GetSidSubAuthority(mandatoryLabel->Label.Sid, subAuthorityCount - 1);
  }

  CloseHandle(processToken);
  return integrityLevel > SECURITY_MANDATORY_MEDIUM_RID ? ovrTrue : ovrFalse;
}
#endif // !defined(_WIN32)

static ModuleHandleType OVR_OpenLibrary(const FilePathCharType* libraryPath, ovrResult* result) {
  *result = ovrSuccess;
#if defined(_WIN32)
  ModuleHandleType hModule = LoadLibraryExW(libraryPath, NULL, LOAD_WITH_ALTERED_SEARCH_PATH);

  if (hModule == NULL) {
    *result = ovrError_LibLoad;
  }

  return hModule;
#else
  // Don't bother trying to dlopen() a file that is not even there.
  if (access(libraryPath, X_OK | R_OK) != 0) {
    *result = ovrError_LibPath;
    return NULL;
  }

  dlerror(); // Clear any previous dlopen() errors

  // Use RTLD_NOW because we don't want unexpected stalls at runtime, and the library isn't very
  // large.
  // Use RTLD_LOCAL to avoid unilaterally exporting resolved symbols to the rest of this process.
  void* lib = dlopen(libraryPath, RTLD_NOW | RTLD_LOCAL);

  if (!lib) {
    fprintf(stderr, "ERROR: Can't load '%s':\n%s\n", libraryPath, dlerror());
  }

  return lib;
#endif
}

static void OVR_CloseLibrary(ModuleHandleType hLibrary) {
  if (hLibrary) {
#if defined(_WIN32)
    // We may need to consider what to do in the case that the library is in an exception state.
    // In a Windows C++ DLL, all global objects (including static members of classes) will be
    // constructed just
    // before the calling of the DllMain with DLL_PROCESS_ATTACH and they will be destroyed just
    // after
    // the call of the DllMain with DLL_PROCESS_DETACH. We may need to intercept DLL_PROCESS_DETACH
    // and
    // have special handling for the case that the DLL is broken.
    FreeLibrary(hLibrary);
#else
    dlclose(hLibrary);
#endif
  }
}

// Returns a valid ModuleHandleType (e.g. Windows HMODULE) or returns ModuleHandleTypeNull (e.g.
// NULL).
// The caller is required to eventually call OVR_CloseLibrary on a valid return handle.
//
static ModuleHandleType OVR_FindLibraryPath(
    int requestedProductVersion,
    int requestedMajorVersion,
    FilePathCharType* libraryPath,
    size_t libraryPathCapacity,
    ovrResult* result) {
  ModuleHandleType moduleHandle;
  FilePathCharType developerDir[OVR_MAX_PATH] = {'\0'};
#if defined(_WIN32)
  FilePathCharType oculusInstallDir[OVR_MAX_PATH] = {'\0'};
#endif // defined(_WIN32)

#if defined(_MSC_VER)
#if defined(_WIN64)
  const char* pBitDepth = "64";
#else
  const char* pBitDepth = "32";
#endif
#elif defined(__APPLE__)
// For Apple platforms we are using a Universal Binary LibOVRRT dylib which has both 32 and 64 in
// it.
#else // Other Unix.
#if defined(__x86_64__)
  const char* pBitDepth = "64";
#else
  const char* pBitDepth = "32";
#endif
#endif

  (void)requestedProductVersion;

  *result = ovrError_LibLoad;
  moduleHandle = ModuleHandleTypeNull;
  if (libraryPathCapacity) {
    libraryPath[0] = '\0';
  }

#if (defined(_MSC_VER) || defined(_WIN32)) && !defined(OVR_FILE_PATH_SEPARATOR)
#define OVR_FILE_PATH_SEPARATOR L"\\"
#else
#define OVR_FILE_PATH_SEPARATOR "/"
#endif

  {
#if defined(_WIN32)
    // Validate we are not high integrity level (i.e. not admin)
    // to avoid pulling env vars set by non-elevated app.
    if (!IsHighIntegrityLevel()) {
      size_t length;
      errno_t success =
          _wgetenv_s(&length, developerDir, _countof(developerDir), L"LIBOVR_DLL_DIR");
      if ((success == 0) && (length != 0) && (length < _countof(developerDir)) &&
          developerDir[length - 2] != OVR_FILE_PATH_SEPARATOR[0]) {
        OVR_wstrlcat(developerDir, OVR_FILE_PATH_SEPARATOR, _countof(developerDir));
      }
    }

    if (OVR_GetOculusBaseDirectory(oculusInstallDir, _countof(oculusInstallDir)) == ovrTrue) {
      size_t baseDirLength = wcslen(oculusInstallDir);

      // If missing a trailing path separator then append one.
      if ((baseDirLength > 0) && (baseDirLength < _countof(oculusInstallDir)) &&
          (oculusInstallDir[baseDirLength - 1] != OVR_FILE_PATH_SEPARATOR[0])) {
        baseDirLength =
            OVR_wstrlcat(oculusInstallDir, OVR_FILE_PATH_SEPARATOR, _countof(oculusInstallDir));
      }

#define OVR_RUNTIME_PATH L"Support\\oculus-runtime\\"

      if ((baseDirLength > 0) &&
          (baseDirLength + _countof(OVR_RUNTIME_PATH) < _countof(oculusInstallDir))) {
        baseDirLength =
            OVR_wstrlcat(oculusInstallDir, OVR_RUNTIME_PATH, _countof(oculusInstallDir));
      }
    }
#else
    // Example value: /dev/OculusSDK/Main/LibOVR/Mac/Debug/
    const char* pLibOvrDllDir = getenv("LIBOVR_DLL_DIR");

    if (pLibOvrDllDir) {
      char developerDir8[OVR_MAX_PATH];
      size_t length = OVR_strlcpy(developerDir8, pLibOvrDllDir, sizeof(developerDir8));

      // If missing a trailing path separator then append one.
      if ((length > 0) && (length < sizeof(developerDir8)) &&
          (developerDir8[length - 1] != OVR_FILE_PATH_SEPARATOR[0])) {
        length = OVR_strlcat(developerDir8, OVR_FILE_PATH_SEPARATOR, sizeof(developerDir8));
      }

      if (length < sizeof(developerDir8)) {
        OVR_strlcpy(developerDir, developerDir8, sizeof(developerDir));
      }
    }
#endif
  }

  {
    size_t i;

#if defined(_WIN32)
    // On Windows, only search the developer directory and the install path
    const FilePathCharType* directoryArray[2];
    directoryArray[0] = developerDir[0] != '\0' ? developerDir : NULL; // Developer directory
    directoryArray[1] = oculusInstallDir;

#elif defined(__APPLE__)
    // https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man1/dyld.1.html

    FilePathCharType homeDir[OVR_MAX_PATH];
    FilePathCharType homeFrameworkDir[OVR_MAX_PATH];
    const FilePathCharType* directoryArray[3];
    size_t homeDirLength = 0;

    const char* pHome = getenv("HOME"); // Try getting the HOME environment variable.

    if (pHome) {
      homeDirLength = OVR_strlcpy(homeDir, pHome, sizeof(homeDir));
    } else {
      // https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man3/getpwuid_r.3.html
      const long pwBufferSize = sysconf(_SC_GETPW_R_SIZE_MAX);

      if (pwBufferSize != -1) {
        char pwBuffer[pwBufferSize];
        struct passwd pw;
        struct passwd* pwResult = NULL;

        if ((getpwuid_r(getuid(), &pw, pwBuffer, pwBufferSize, &pwResult) == 0) && pwResult)
          homeDirLength = OVR_strlcpy(homeDir, pw.pw_dir, sizeof(homeDir));
      }
    }

    if (homeDirLength) {
      if (homeDir[homeDirLength - 1] == '/')
        homeDir[homeDirLength - 1] = '\0';
      OVR_strlcpy(homeFrameworkDir, homeDir, sizeof(homeFrameworkDir));
      OVR_strlcat(homeFrameworkDir, "/Library/Frameworks/", sizeof(homeFrameworkDir));
    } else {
      homeFrameworkDir[0] = '\0';
    }

    directoryArray[0] = developerDir[0] != '\0' ? developerDir : NULL; // Developer directory.
    directoryArray[1] = homeFrameworkDir; // ~/Library/Frameworks/
    directoryArray[2] = "/Library/Frameworks/"; // DYLD_FALLBACK_FRAMEWORK_PATH

#else
    const FilePathCharType* directoryArray[3];
    directoryArray[0] = developerDir[0] != '\0' ? developerDir : NULL; // Developer directory.
    directoryArray[1] = "/usr/local/lib/";
    directoryArray[2] = "/usr/lib/";
#endif

    // Versioned file expectations.
    //     Windows: LibOVRRT<BIT_DEPTH>_<PRODUCT_VERSION>_<MAJOR_VERSION>.dll
    //     // Example: LibOVRRT64_1_1.dll -- LibOVRRT 64 bit, product 1, major version 1,
    //     minor/patch/build numbers unspecified in the name.
    //     Mac:
    //     LibOVRRT_<PRODUCT_VERSION>.framework/Versions/<MAJOR_VERSION>/LibOVRRT_<PRODUCT_VERSION>
    //     // We are not presently using the .framework bundle's Current directory to hold the
    //     version number. This may change.
    //     Linux:   libOVRRT<BIT_DEPTH>_<PRODUCT_VERSION>.so.<MAJOR_VERSION>
    //     // The file on disk may contain a minor version number, but a symlink is used to map this
    //     major-only version to it.

    // Since we are manually loading the LibOVR dynamic library, we need to look in various
    // locations for a file
    // that matches our requirements. The functionality required is somewhat similar to the
    // operating system's
    // dynamic loader functionality. Each OS has some differences in how this is handled.
    // Future versions of this may iterate over all libOVRRT.so.* files in the directory and use the
    // one that matches our requirements.
    //
    // We need to look for a library that matches the product version and major version of the
    // caller's request,
    // and that library needs to support a minor version that is >= the requested minor version.
    // Currently we
    // don't test the minor version here, as the library is named based only on the product and
    // major version.
    // Currently the minor version test is handled via the initialization of the library and the
    // initialization
    // fails if minor version cannot be supported by the library. The reason this is done during
    // initialization
    // is that the library can at runtime support multiple minor versions based on the user's
    // request. To the
    // external user, all that matters it that they call ovr_Initialize with a requested version and
    // it succeeds
    // or fails.
    //
    // The product version is something that is at a higher level than the major version, and is not
    // something that's
    // always seen in libraries (an example is the well-known LibXml2 library, in which the 2 is
    // essentially the product version).

    for (i = 0; i < sizeof(directoryArray) / sizeof(directoryArray[0]); ++i) {
      if (directoryArray[i] != NULL) {
#if defined(_WIN32)
        int printfResult = swprintf(
            libraryPath,
            libraryPathCapacity,
            L"%lsLibOVRRT%hs_%d.dll",
            directoryArray[i],
            pBitDepth,
            requestedMajorVersion);
#elif defined(__APPLE__)
        (void)requestedMajorVersion;
        // https://developer.apple.com/library/mac/documentation/MacOSX/Conceptual/BPFrameworks/Concepts/VersionInformation.html
        // Macintosh application bundles have the option of embedding dependent frameworks within
        // the application bundle itself. A problem with that is that it doesn't support
        // vendor-supplied updates to the framework.
        int printfResult =
            snprintf(libraryPath, libraryPathCapacity, "%sLibOVRRT.dylib", directoryArray[i]);

#else // Unix
      // Applications that depend on the OS (e.g. ld-linux / ldd) can rely on the library being in a
      // common location
      // such as /usr/lib or can rely on the -rpath linker option to embed a path for the OS to
      // check for the library,
      // or can rely on the LD_LIBRARY_PATH environment variable being set. It's generally not
      // recommended that applications
      // depend on LD_LIBRARY_PATH be globally modified, partly due to potentialy security issues.
      // Currently we check the current application directory, current working directory, and then
      // /usr/lib and possibly others.
        int printfResult = snprintf(
            libraryPath,
            libraryPathCapacity,
            "%slibOVRRT%s.so.%d",
            directoryArray[i],
            pBitDepth,
            requestedMajorVersion);
#endif

        if ((printfResult >= 0) && (printfResult < (int)libraryPathCapacity)) {
          moduleHandle = OVR_OpenLibrary(libraryPath, result);
          if (moduleHandle != ModuleHandleTypeNull)
            return moduleHandle;
        }
      }
    }
  }

  return moduleHandle;
}

//-----------------------------------------------------------------------------------
// ***** hLibOVR
//
// global handle to the LivOVR shared library.
//
static ModuleHandleType hLibOVR = NULL;

// This function is currently unsupported.
ModuleHandleType ovr_GetLibOVRRTHandle() {
  return hLibOVR;
}

//-----------------------------------------------------------------------------------
// ***** Function declarations
//

//-----------------------------------------------------------------------------------
// ***** OVR_DECLARE_IMPORT
//
// Creates a pointer and loader value union for each entry in OVR_LIST_APIS()
//

#define OVR_DECLARE_IMPORT(ReturnValue, FunctionName, OptionalVersion, Arguments) \
  union {                                                                         \
    ReturnValue(OVR_CDECL* Ptr) Arguments;                                        \
    ModuleFunctionType Symbol;                                                    \
  } FunctionName;

#define OVR_IGNORE_IMPORT(ReturnValue, FunctionName, OptionalVersion, Arguments)

//-----------------------------------------------------------------------------------
// ***** API - a structure with each API entrypoint as a FunctionName.Ptr and FunctionName.Symbol
// union
//

static struct { OVR_LIST_APIS(OVR_DECLARE_IMPORT, OVR_IGNORE_IMPORT) } API = {{NULL}};

static void OVR_UnloadSharedLibrary() {
  memset(&API, 0, sizeof(API));
  if (hLibOVR)
    OVR_CloseLibrary(hLibOVR);
  hLibOVR = NULL;
}

// Don't put this on the heap
static ovrErrorInfo LastInitializeErrorInfo = {
    ovrError_NotInitialized,
    "ovr_Initialize never called"};

static ovrResult OVR_LoadSharedLibrary(int requestedProductVersion, int requestedMajorVersion) {
  FilePathCharType filePath[OVR_MAX_PATH];
  const char* SymbolName = NULL;
  ovrResult result = ovrSuccess;

  if (hLibOVR)
    return result;

  hLibOVR = OVR_FindLibraryPath(
      requestedProductVersion,
      requestedMajorVersion,
      filePath,
      sizeof(filePath) / sizeof(filePath[0]),
      &result);

  if (!hLibOVR) {
    LastInitializeErrorInfo.Result = result;
    OVR_strlcpy(
        LastInitializeErrorInfo.ErrorString,
        "Unable to load LibOVRRT DLL",
        sizeof(LastInitializeErrorInfo.ErrorString));
    return result;
  }

  ovrBool AllowSymbolLoadFailure = ovrFalse;

  // Zero the API table just to be paranoid
  memset(&API, 0, sizeof(API));

// Load the current API entrypoint using the catenated FunctionName and OptionalVersion
#define OVR_GETFUNCTION(ReturnValue, FunctionName, OptionalVersion, Arguments) \
  SymbolName = #FunctionName #OptionalVersion;                                 \
  API.FunctionName.Symbol = OVR_DLSYM(hLibOVR, SymbolName);                    \
  if (!API.FunctionName.Symbol && !AllowSymbolLoadFailure) {                   \
    LastInitializeErrorInfo.Result = result = ovrError_LibSymbols;             \
    OVR_strlcpy(                                                               \
        LastInitializeErrorInfo.ErrorString,                                   \
        "Unable to locate LibOVRRT symbol: ",                                  \
        sizeof(LastInitializeErrorInfo.ErrorString));                          \
    OVR_strlcat(                                                               \
        LastInitializeErrorInfo.ErrorString,                                   \
        SymbolName,                                                            \
        sizeof(LastInitializeErrorInfo.ErrorString));                          \
    goto FailedToLoadSymbol;                                                   \
  }

  OVR_LIST_APIS(OVR_GETFUNCTION, OVR_IGNORE_IMPORT)

#undef OVR_GETFUNCTION

  return result;

FailedToLoadSymbol:
  // Check SymbolName for the name of the API which failed to load
  OVR_UnloadSharedLibrary();
  return result;
}

// These defaults are also in CAPI.cpp
static const ovrInitParams DefaultParams = {
    ovrInit_RequestVersion, // Flags
    OVR_MINOR_VERSION, // RequestedMinorVersion
    0, // LogCallback
    0, // UserData
    0, // ConnectionTimeoutSeconds
    OVR_ON64("") // pad0
};

OVR_PUBLIC_FUNCTION(ovrResult) ovr_Initialize(const ovrInitParams* inputParams) {
  ovrResult result;
  ovrInitParams params;

  typedef void(OVR_CDECL * ovr_ReportClientInfoType)(
      unsigned int compilerVersion,
      int productVersion,
      int majorVersion,
      int minorVersion,
      int patchVersion,
      int buildNumber);
  ovr_ReportClientInfoType reportClientInfo;

  // Do something with our version signature hash to prevent
  // it from being optimized out. In this case, compute
  // a cheap CRC.
  uint8_t crc = 0;
  size_t i;

  for (i = 0; i < (sizeof(OculusSDKUniqueIdentifier) - 3);
       ++i) // Minus 3 because we have trailing OVR_MAJOR_VERSION, OVR_MINOR_VERSION,
  // OVR_PATCH_VERSION which vary per version.
  {
    crc ^= OculusSDKUniqueIdentifier[i];
  }

  assert(crc == OculusSDKUniqueIdentifierXORResult);
  if (crc != OculusSDKUniqueIdentifierXORResult) {
    return ovrError_Initialize;
  }

  if (!inputParams) {
    params = DefaultParams;
  } else {
    params = *inputParams;

    // If not requesting a particular minor version,
    if (!(params.Flags & ovrInit_RequestVersion)) {
      // Enable requesting the default minor version.
      params.Flags |= ovrInit_RequestVersion;
      params.RequestedMinorVersion = OVR_MINOR_VERSION;
    }
  }

  // Clear non-writable bits provided by client code.
  params.Flags &= ovrinit_WritableBits;


  // Error out if the requested minor version is less than our lowest deemed compatible version
  // denoted by OVR_MIN_REQUESTABLE_MINOR_VERSION.
  // Note: This code has to be in the shim as we want to enforce usage of the new API versions for
  // applications being recompiled while maintaining backwards compatibility with older apps
  if (params.RequestedMinorVersion < OVR_MIN_REQUESTABLE_MINOR_VERSION) {
    // Requested LibOVRRT version too low
    result = ovrError_LibVersion;
    return result;
  }

  // By design we ignore the build version in the library search.
  result = OVR_LoadSharedLibrary(OVR_PRODUCT_VERSION, OVR_MAJOR_VERSION);
  if (result != ovrSuccess)
    return result;

  result = API.ovr_Initialize.Ptr(&params);

  if (result != ovrSuccess) {
    // Stash the last initialization error for the shim to return if
    // ovr_GetLastErrorInfo is called after we unload the dll below
    if (API.ovr_GetLastErrorInfo.Ptr) {
      API.ovr_GetLastErrorInfo.Ptr(&LastInitializeErrorInfo);
    }
    OVR_UnloadSharedLibrary();
  }

  reportClientInfo =
      (ovr_ReportClientInfoType)(uintptr_t)OVR_DLSYM(hLibOVR, "ovr_ReportClientInfo");

  if (reportClientInfo) {
    unsigned int mscFullVer = 0;
#if defined(_MSC_FULL_VER)
    mscFullVer = _MSC_FULL_VER;
#endif // _MSC_FULL_VER

    reportClientInfo(
        mscFullVer,
        OVR_PRODUCT_VERSION,
        OVR_MAJOR_VERSION,
        OVR_MINOR_VERSION,
        OVR_PATCH_VERSION,
        0);
  }

  return result;
}

OVR_PUBLIC_FUNCTION(void) ovr_Shutdown() {
  if (!API.ovr_Shutdown.Ptr)
    return;
  API.ovr_Shutdown.Ptr();
  OVR_UnloadSharedLibrary();
}

OVR_PUBLIC_FUNCTION(const char*) ovr_GetVersionString() {
  // We don't directly return the value of the DLL API.ovr_GetVersionString.Ptr call,
  // because that call returns a pointer to memory within the DLL. If the DLL goes
  // away then that pointer becomes invalid while the process may still be holding
  // onto it. So we save a local copy of it which is always valid.
  static char dllVersionStringLocal[32];
  const char* dllVersionString;

  if (!API.ovr_GetVersionString.Ptr)
    return "(Unable to load LibOVR)";

  dllVersionString = API.ovr_GetVersionString.Ptr(); // Guaranteed to always be valid.
  assert(dllVersionString != NULL);
  OVR_strlcpy(dllVersionStringLocal, dllVersionString, sizeof(dllVersionStringLocal));

  return dllVersionStringLocal;
}

OVR_PUBLIC_FUNCTION(void) ovr_GetLastErrorInfo(ovrErrorInfo* errorInfo) {
  if (!API.ovr_GetLastErrorInfo.Ptr) {
    *errorInfo = LastInitializeErrorInfo;
  } else
    API.ovr_GetLastErrorInfo.Ptr(errorInfo);
}

OVR_PUBLIC_FUNCTION(ovrHmdDesc) ovr_GetHmdDesc(ovrSession session) {
  if (!API.ovr_GetHmdDesc.Ptr) {
    ovrHmdDesc hmdDesc;
    memset(&hmdDesc, 0, sizeof(hmdDesc));
    hmdDesc.Type = ovrHmd_None;
    return hmdDesc;
  }

  return API.ovr_GetHmdDesc.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrHmdColorDesc) ovr_GetHmdColorDesc(ovrSession session) {
  if (!API.ovr_GetHmdColorDesc.Ptr) {
    ovrHmdColorDesc hmdColorDesc;
    memset(&hmdColorDesc, 0, sizeof(hmdColorDesc));
    return hmdColorDesc;
  }

  return API.ovr_GetHmdColorDesc.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SetClientColorDesc(ovrSession session, const ovrHmdColorDesc* hmdColorDesc) {
  if (!API.ovr_SetClientColorDesc.Ptr) {
    return ovrError_NotInitialized;
  }

  return API.ovr_SetClientColorDesc.Ptr(session, hmdColorDesc);
}

OVR_PUBLIC_FUNCTION(unsigned int) ovr_GetTrackerCount(ovrSession session) {
  if (!API.ovr_GetTrackerCount.Ptr) {
    return 0;
  }

  return API.ovr_GetTrackerCount.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrTrackerDesc)
ovr_GetTrackerDesc(ovrSession session, unsigned int trackerDescIndex) {
  if (!API.ovr_GetTrackerDesc.Ptr) {
    ovrTrackerDesc trackerDesc;
    memset(&trackerDesc, 0, sizeof(trackerDesc));
    return trackerDesc;
  }

  return API.ovr_GetTrackerDesc.Ptr(session, trackerDescIndex);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_Create(ovrSession* pSession, ovrGraphicsLuid* pLuid) {
  if (!API.ovr_Create.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_Create.Ptr(pSession, pLuid);
}

OVR_PUBLIC_FUNCTION(void) ovr_Destroy(ovrSession session) {
  if (!API.ovr_Destroy.Ptr)
    return;
  API.ovr_Destroy.Ptr(session);
}


OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetSessionStatus(ovrSession session, ovrSessionStatus* sessionStatus) {
  if (!API.ovr_GetSessionStatus.Ptr) {
    if (sessionStatus) {
      sessionStatus->IsVisible = ovrFalse;
      sessionStatus->HmdPresent = ovrFalse;
      sessionStatus->HmdMounted = ovrFalse;
      sessionStatus->ShouldQuit = ovrFalse;
      sessionStatus->DisplayLost = ovrFalse;
      sessionStatus->ShouldRecenter = ovrFalse;
      sessionStatus->HasInputFocus = ovrFalse;
      sessionStatus->OverlayPresent = ovrFalse;
      sessionStatus->DepthRequested = ovrFalse;
    }

    return ovrError_NotInitialized;
  }

  return API.ovr_GetSessionStatus.Ptr(session, sessionStatus);
}


OVR_PUBLIC_FUNCTION(ovrResult)
ovr_IsExtensionSupported(ovrSession session, ovrExtensions extension, ovrBool* extensionSupported) {
  if (!API.ovr_IsExtensionSupported.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_IsExtensionSupported.Ptr(session, extension, extensionSupported);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_EnableExtension(ovrSession session, ovrExtensions extension) {
  if (!API.ovr_EnableExtension.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_EnableExtension.Ptr(session, extension);
}


OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SetTrackingOriginType(ovrSession session, ovrTrackingOrigin origin) {
  if (!API.ovr_SetTrackingOriginType.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_SetTrackingOriginType.Ptr(session, origin);
}

OVR_PUBLIC_FUNCTION(ovrTrackingOrigin) ovr_GetTrackingOriginType(ovrSession session) {
  if (!API.ovr_GetTrackingOriginType.Ptr)
    return ovrTrackingOrigin_EyeLevel;
  return API.ovr_GetTrackingOriginType.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_RecenterTrackingOrigin(ovrSession session) {
  if (!API.ovr_RecenterTrackingOrigin.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_RecenterTrackingOrigin.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_SpecifyTrackingOrigin(ovrSession session, ovrPosef originPose) {
  if (!API.ovr_SpecifyTrackingOrigin.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_SpecifyTrackingOrigin.Ptr(session, originPose);
}

OVR_PUBLIC_FUNCTION(void) ovr_ClearShouldRecenterFlag(ovrSession session) {
  if (!API.ovr_ClearShouldRecenterFlag.Ptr)
    return;
  API.ovr_ClearShouldRecenterFlag.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrTrackingState)
ovr_GetTrackingState(ovrSession session, double absTime, ovrBool latencyMarker) {
  if (!API.ovr_GetTrackingState.Ptr) {
    ovrTrackingState nullTrackingState;
    memset(&nullTrackingState, 0, sizeof(nullTrackingState));
    return nullTrackingState;
  }

  return API.ovr_GetTrackingState.Ptr(session, absTime, latencyMarker);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetDevicePoses(
    ovrSession session,
    ovrTrackedDeviceType* deviceTypes,
    int deviceCount,
    double absTime,
    ovrPoseStatef* outDevicePoses) {
  if (!API.ovr_GetDevicePoses.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_GetDevicePoses.Ptr(session, deviceTypes, deviceCount, absTime, outDevicePoses);
}


OVR_PUBLIC_FUNCTION(ovrTrackerPose)
ovr_GetTrackerPose(ovrSession session, unsigned int trackerPoseIndex) {
  if (!API.ovr_GetTrackerPose.Ptr) {
    ovrTrackerPose nullTrackerPose;
    memset(&nullTrackerPose, 0, sizeof(nullTrackerPose));
    return nullTrackerPose;
  }

  return API.ovr_GetTrackerPose.Ptr(session, trackerPoseIndex);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetInputState(ovrSession session, ovrControllerType controllerType, ovrInputState* inputState) {
  if (!API.ovr_GetInputState.Ptr) {
    if (inputState)
      memset(inputState, 0, sizeof(ovrInputState));
    return ovrError_NotInitialized;
  }
  return API.ovr_GetInputState.Ptr(session, controllerType, inputState);
}


OVR_PUBLIC_FUNCTION(unsigned int) ovr_GetConnectedControllerTypes(ovrSession session) {
  if (!API.ovr_GetConnectedControllerTypes.Ptr) {
    return 0;
  }
  return API.ovr_GetConnectedControllerTypes.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrTouchHapticsDesc)
ovr_GetTouchHapticsDesc(ovrSession session, ovrControllerType controllerType) {
  if (!API.ovr_GetTouchHapticsDesc.Ptr) {
    ovrTouchHapticsDesc nullDesc;
    memset(&nullDesc, 0, sizeof(nullDesc));
    return nullDesc;
  }

  return API.ovr_GetTouchHapticsDesc.Ptr(session, controllerType);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SetControllerVibration(
    ovrSession session,
    ovrControllerType controllerType,
    float frequency,
    float amplitude) {
  if (!API.ovr_SetControllerVibration.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_SetControllerVibration.Ptr(session, controllerType, frequency, amplitude);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SubmitControllerVibration(
    ovrSession session,
    ovrControllerType controllerType,
    const ovrHapticsBuffer* buffer) {
  if (!API.ovr_SubmitControllerVibration.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_SubmitControllerVibration.Ptr(session, controllerType, buffer);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetControllerVibrationState(
    ovrSession session,
    ovrControllerType controllerType,
    ovrHapticsPlaybackState* outState) {
  if (!API.ovr_GetControllerVibrationState.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetControllerVibrationState.Ptr(session, controllerType, outState);
}



OVR_PUBLIC_FUNCTION(ovrResult)
ovr_TestBoundary(
    ovrSession session,
    ovrTrackedDeviceType deviceBitmask,
    ovrBoundaryType singleBoundaryType,
    ovrBoundaryTestResult* outTestResult) {
  if (!API.ovr_TestBoundary.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_TestBoundary.Ptr(session, deviceBitmask, singleBoundaryType, outTestResult);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_TestBoundaryPoint(
    ovrSession session,
    const ovrVector3f* point,
    ovrBoundaryType singleBoundaryType,
    ovrBoundaryTestResult* outTestResult) {
  if (!API.ovr_TestBoundaryPoint.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_TestBoundaryPoint.Ptr(session, point, singleBoundaryType, outTestResult);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SetBoundaryLookAndFeel(ovrSession session, const ovrBoundaryLookAndFeel* lookAndFeel) {
  if (!API.ovr_SetBoundaryLookAndFeel.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_SetBoundaryLookAndFeel.Ptr(session, lookAndFeel);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_ResetBoundaryLookAndFeel(ovrSession session) {
  if (!API.ovr_ResetBoundaryLookAndFeel.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_ResetBoundaryLookAndFeel.Ptr(session);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetBoundaryGeometry(
    ovrSession session,
    ovrBoundaryType singleBoundaryType,
    ovrVector3f* outFloorPoints,
    int* outFloorPointsCount) {
  if (!API.ovr_GetBoundaryGeometry.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetBoundaryGeometry.Ptr(
      session, singleBoundaryType, outFloorPoints, outFloorPointsCount);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetBoundaryDimensions(
    ovrSession session,
    ovrBoundaryType singleBoundaryType,
    ovrVector3f* outDimensions) {
  if (!API.ovr_GetBoundaryDimensions.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetBoundaryDimensions.Ptr(session, singleBoundaryType, outDimensions);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetBoundaryVisible(ovrSession session, ovrBool* outIsVisible) {
  if (!API.ovr_GetBoundaryVisible.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetBoundaryVisible.Ptr(session, outIsVisible);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_RequestBoundaryVisible(ovrSession session, ovrBool visible) {
  if (!API.ovr_RequestBoundaryVisible.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_RequestBoundaryVisible.Ptr(session, visible);
}

OVR_PUBLIC_FUNCTION(ovrSizei)
ovr_GetFovTextureSize(
    ovrSession session,
    ovrEyeType eye,
    ovrFovPort fov,
    float pixelsPerDisplayPixel) {
  if (!API.ovr_GetFovTextureSize.Ptr) {
    ovrSizei nullSize;
    memset(&nullSize, 0, sizeof(nullSize));
    return nullSize;
  }

  return API.ovr_GetFovTextureSize.Ptr(session, eye, fov, pixelsPerDisplayPixel);
}

#if defined(_WIN32)
OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateTextureSwapChainDX(
    ovrSession session,
    IUnknown* d3dPtr,
    const ovrTextureSwapChainDesc* desc,
    ovrTextureSwapChain* outTextureSet) {
  if (!API.ovr_CreateTextureSwapChainDX.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateTextureSwapChainDX.Ptr(session, d3dPtr, desc, outTextureSet);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateMirrorTextureDX(
    ovrSession session,
    IUnknown* d3dPtr,
    const ovrMirrorTextureDesc* desc,
    ovrMirrorTexture* outMirrorTexture) {
  if (!API.ovr_CreateMirrorTextureDX.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateMirrorTextureDX.Ptr(session, d3dPtr, desc, outMirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateMirrorTextureWithOptionsDX(
    ovrSession session,
    IUnknown* d3dPtr,
    const ovrMirrorTextureDesc* desc,
    ovrMirrorTexture* outMirrorTexture) {
  if (!API.ovr_CreateMirrorTextureWithOptionsDX.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateMirrorTextureWithOptionsDX.Ptr(session, d3dPtr, desc, outMirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainBufferDX(
    ovrSession session,
    ovrTextureSwapChain chain,
    int index,
    IID iid,
    void** ppObject) {
  if (!API.ovr_GetTextureSwapChainBufferDX.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainBufferDX.Ptr(session, chain, index, iid, ppObject);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetMirrorTextureBufferDX(
    ovrSession session,
    ovrMirrorTexture mirror,
    IID iid,
    void** ppObject) {
  if (!API.ovr_GetMirrorTextureBufferDX.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetMirrorTextureBufferDX.Ptr(session, mirror, iid, ppObject);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceOutWaveId(unsigned int* deviceOutId) {
  if (!API.ovr_GetAudioDeviceOutWaveId.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceOutWaveId.Ptr(deviceOutId);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceInWaveId(unsigned int* deviceInId) {
  if (!API.ovr_GetAudioDeviceInWaveId.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceInWaveId.Ptr(deviceInId);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceOutGuidStr(WCHAR* deviceOutStrBuffer) {
  if (!API.ovr_GetAudioDeviceOutGuidStr.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceOutGuidStr.Ptr(deviceOutStrBuffer);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceOutGuid(GUID* deviceOutGuid) {
  if (!API.ovr_GetAudioDeviceOutGuid.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceOutGuid.Ptr(deviceOutGuid);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceInGuidStr(WCHAR* deviceInStrBuffer) {
  if (!API.ovr_GetAudioDeviceInGuidStr.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceInGuidStr.Ptr(deviceInStrBuffer);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetAudioDeviceInGuid(GUID* deviceInGuid) {
  if (!API.ovr_GetAudioDeviceInGuid.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetAudioDeviceInGuid.Ptr(deviceInGuid);
}

#endif // _WIN32

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateTextureSwapChainGL(
    ovrSession session,
    const ovrTextureSwapChainDesc* desc,
    ovrTextureSwapChain* outTextureSet) {
  if (!API.ovr_CreateTextureSwapChainGL.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateTextureSwapChainGL.Ptr(session, desc, outTextureSet);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateMirrorTextureGL(
    ovrSession session,
    const ovrMirrorTextureDesc* desc,
    ovrMirrorTexture* outMirrorTexture) {
  if (!API.ovr_CreateMirrorTextureGL.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateMirrorTextureGL.Ptr(session, desc, outMirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateMirrorTextureWithOptionsGL(
    ovrSession session,
    const ovrMirrorTextureDesc* desc,
    ovrMirrorTexture* outMirrorTexture) {
  if (!API.ovr_CreateMirrorTextureWithOptionsGL.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateMirrorTextureWithOptionsGL.Ptr(session, desc, outMirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainBufferGL(
    ovrSession session,
    ovrTextureSwapChain chain,
    int index,
    unsigned int* texId) {
  if (!API.ovr_GetTextureSwapChainBufferGL.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainBufferGL.Ptr(session, chain, index, texId);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetMirrorTextureBufferGL(ovrSession session, ovrMirrorTexture mirror, unsigned int* texId) {
  if (!API.ovr_GetMirrorTextureBufferGL.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetMirrorTextureBufferGL.Ptr(session, mirror, texId);
}

#if defined(_WIN32)
OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetInstanceExtensionsVk(
    ovrGraphicsLuid luid,
    char* extensionNames,
    uint32_t* inoutExtensionNamesSize) {
  if (!API.ovr_GetInstanceExtensionsVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetInstanceExtensionsVk.Ptr(luid, extensionNames, inoutExtensionNamesSize);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetDeviceExtensionsVk(
    ovrGraphicsLuid luid,
    char* extensionNames,
    uint32_t* inoutExtensionNamesSize) {
  if (!API.ovr_GetDeviceExtensionsVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetDeviceExtensionsVk.Ptr(luid, extensionNames, inoutExtensionNamesSize);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetSessionPhysicalDeviceVk(
    ovrSession session,
    ovrGraphicsLuid luid,
    VkInstance instance,
    VkPhysicalDevice* out_physicalDevice) {
  if (!API.ovr_GetSessionPhysicalDeviceVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetSessionPhysicalDeviceVk.Ptr(session, luid, instance, out_physicalDevice);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_SetSynchronizationQueueVk(ovrSession session, VkQueue queue) {
  if (!API.ovr_SetSynchronizationQueueVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_SetSynchronizationQueueVk.Ptr(session, queue);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateTextureSwapChainVk(
    ovrSession session,
    VkDevice device,
    const ovrTextureSwapChainDesc* desc,
    ovrTextureSwapChain* out_TextureSwapChain) {
  if (!API.ovr_CreateTextureSwapChainVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateTextureSwapChainVk.Ptr(session, device, desc, out_TextureSwapChain);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainBufferVk(
    ovrSession session,
    ovrTextureSwapChain chain,
    int index,
    VkImage* out_Image) {
  if (!API.ovr_GetTextureSwapChainBufferVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainBufferVk.Ptr(session, chain, index, out_Image);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CreateMirrorTextureWithOptionsVk(
    ovrSession session,
    VkDevice device,
    const ovrMirrorTextureDesc* desc,
    ovrMirrorTexture* out_MirrorTexture) {
  if (!API.ovr_CreateMirrorTextureWithOptionsVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CreateMirrorTextureWithOptionsVk.Ptr(session, device, desc, out_MirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetMirrorTextureBufferVk(
    ovrSession session,
    ovrMirrorTexture mirrorTexture,
    VkImage* out_Image) {
  if (!API.ovr_GetMirrorTextureBufferVk.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetMirrorTextureBufferVk.Ptr(session, mirrorTexture, out_Image);
}
#endif // defined (_WIN32)

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainLength(ovrSession session, ovrTextureSwapChain chain, int* length) {
  if (!API.ovr_GetTextureSwapChainLength.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainLength.Ptr(session, chain, length);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainCurrentIndex(
    ovrSession session,
    ovrTextureSwapChain chain,
    int* currentIndex) {
  if (!API.ovr_GetTextureSwapChainCurrentIndex.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainCurrentIndex.Ptr(session, chain, currentIndex);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetTextureSwapChainDesc(
    ovrSession session,
    ovrTextureSwapChain chain,
    ovrTextureSwapChainDesc* desc) {
  if (!API.ovr_GetTextureSwapChainDesc.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetTextureSwapChainDesc.Ptr(session, chain, desc);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_CommitTextureSwapChain(ovrSession session, ovrTextureSwapChain chain) {
  if (!API.ovr_CommitTextureSwapChain.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_CommitTextureSwapChain.Ptr(session, chain);
}

OVR_PUBLIC_FUNCTION(void)
ovr_DestroyTextureSwapChain(ovrSession session, ovrTextureSwapChain chain) {
  if (!API.ovr_DestroyTextureSwapChain.Ptr)
    return;

  API.ovr_DestroyTextureSwapChain.Ptr(session, chain);
}

OVR_PUBLIC_FUNCTION(void)
ovr_DestroyMirrorTexture(ovrSession session, ovrMirrorTexture mirrorTexture) {
  if (!API.ovr_DestroyMirrorTexture.Ptr)
    return;

  API.ovr_DestroyMirrorTexture.Ptr(session, mirrorTexture);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetFovStencil(
    ovrSession session,
    const ovrFovStencilDesc* fovStencilDesc,
    ovrFovStencilMeshBuffer* meshBuffer) {
  if (!API.ovr_GetFovStencil.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetFovStencil.Ptr(session, fovStencilDesc, meshBuffer);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_WaitToBeginFrame(ovrSession session, long long frameIndex) {
  if (!API.ovr_WaitToBeginFrame.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_WaitToBeginFrame.Ptr(session, frameIndex);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_BeginFrame(ovrSession session, long long frameIndex) {
  if (!API.ovr_BeginFrame.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_BeginFrame.Ptr(session, frameIndex);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_EndFrame(
    ovrSession session,
    long long frameIndex,
    const ovrViewScaleDesc* viewScaleDesc,
    ovrLayerHeader const* const* layerPtrList,
    unsigned int layerCount) {
  if (!API.ovr_EndFrame.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_EndFrame.Ptr(session, frameIndex, viewScaleDesc, layerPtrList, layerCount);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SubmitFrame(
    ovrSession session,
    long long frameIndex,
    const ovrViewScaleDesc* viewScaleDesc,
    ovrLayerHeader const* const* layerPtrList,
    unsigned int layerCount) {
  if (!API.ovr_SubmitFrame.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_SubmitFrame.Ptr(session, frameIndex, viewScaleDesc, layerPtrList, layerCount);
}


OVR_PUBLIC_FUNCTION(ovrEyeRenderDesc)
ovr_GetRenderDesc(ovrSession session, ovrEyeType eyeType, ovrFovPort fov) {
  if (!API.ovr_GetRenderDesc.Ptr) {
    ovrEyeRenderDesc nullEyeRenderDesc;
    memset(&nullEyeRenderDesc, 0, sizeof(nullEyeRenderDesc));
    return nullEyeRenderDesc;
  }
  return API.ovr_GetRenderDesc.Ptr(session, eyeType, fov);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_GetPerfStats(ovrSession session, ovrPerfStats* outPerfStats) {
  if (!API.ovr_GetPerfStats.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_GetPerfStats.Ptr(session, outPerfStats);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_ResetPerfStats(ovrSession session) {
  if (!API.ovr_ResetPerfStats.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_ResetPerfStats.Ptr(session);
}

OVR_PUBLIC_FUNCTION(double) ovr_GetPredictedDisplayTime(ovrSession session, long long frameIndex) {
  if (!API.ovr_GetPredictedDisplayTime.Ptr)
    return 0.0;

  return API.ovr_GetPredictedDisplayTime.Ptr(session, frameIndex);
}

OVR_PUBLIC_FUNCTION(double) ovr_GetTimeInSeconds() {
  if (!API.ovr_GetTimeInSeconds.Ptr)
    return 0.;
  return API.ovr_GetTimeInSeconds.Ptr();
}

OVR_PUBLIC_FUNCTION(ovrBool)
ovr_GetBool(ovrSession session, const char* propertyName, ovrBool defaultVal) {
  if (!API.ovr_GetBool.Ptr)
    return ovrFalse;
  return API.ovr_GetBool.Ptr(session, propertyName, defaultVal);
}

OVR_PUBLIC_FUNCTION(ovrBool)
ovr_SetBool(ovrSession session, const char* propertyName, ovrBool value) {
  if (!API.ovr_SetBool.Ptr)
    return ovrFalse;
  return API.ovr_SetBool.Ptr(session, propertyName, value);
}

OVR_PUBLIC_FUNCTION(int) ovr_GetInt(ovrSession session, const char* propertyName, int defaultVal) {
  if (!API.ovr_GetInt.Ptr)
    return 0;
  return API.ovr_GetInt.Ptr(session, propertyName, defaultVal);
}

OVR_PUBLIC_FUNCTION(ovrBool) ovr_SetInt(ovrSession session, const char* propertyName, int value) {
  if (!API.ovr_SetInt.Ptr)
    return ovrFalse;
  return API.ovr_SetInt.Ptr(session, propertyName, value);
}

OVR_PUBLIC_FUNCTION(float)
ovr_GetFloat(ovrSession session, const char* propertyName, float defaultVal) {
  if (!API.ovr_GetFloat.Ptr)
    return 0.f;
  return API.ovr_GetFloat.Ptr(session, propertyName, defaultVal);
}

OVR_PUBLIC_FUNCTION(ovrBool)
ovr_SetFloat(ovrSession session, const char* propertyName, float value) {
  if (!API.ovr_SetFloat.Ptr)
    return ovrFalse;
  return API.ovr_SetFloat.Ptr(session, propertyName, value);
}

OVR_PUBLIC_FUNCTION(unsigned int)
ovr_GetFloatArray(
    ovrSession session,
    const char* propertyName,
    float values[],
    unsigned int arraySize) {
  if (!API.ovr_GetFloatArray.Ptr)
    return 0;
  return API.ovr_GetFloatArray.Ptr(session, propertyName, values, arraySize);
}

OVR_PUBLIC_FUNCTION(ovrBool)
ovr_SetFloatArray(
    ovrSession session,
    const char* propertyName,
    const float values[],
    unsigned int arraySize) {
  if (!API.ovr_SetFloatArray.Ptr)
    return ovrFalse;
  return API.ovr_SetFloatArray.Ptr(session, propertyName, values, arraySize);
}

OVR_PUBLIC_FUNCTION(const char*)
ovr_GetString(ovrSession session, const char* propertyName, const char* defaultVal) {
  if (!API.ovr_GetString.Ptr)
    return "(Unable to load LibOVR)";
  return API.ovr_GetString.Ptr(session, propertyName, defaultVal);
}

OVR_PUBLIC_FUNCTION(ovrBool)
ovr_SetString(ovrSession session, const char* propertyName, const char* value) {
  if (!API.ovr_SetString.Ptr)
    return ovrFalse;
  return API.ovr_SetString.Ptr(session, propertyName, value);
}

OVR_PUBLIC_FUNCTION(int) ovr_TraceMessage(int level, const char* message) {
  if (!API.ovr_TraceMessage.Ptr)
    return -1;

  return API.ovr_TraceMessage.Ptr(level, message);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_IdentifyClient(const char* identity) {
  if (!API.ovr_IdentifyClient.Ptr)
    return ovrError_NotInitialized;

  return API.ovr_IdentifyClient.Ptr(identity);
}

OVR_PUBLIC_FUNCTION(ovrResult) ovr_Lookup(const char* name, void** data) {
  if (!API.ovr_Lookup.Ptr)
    return ovrError_NotInitialized;
  return API.ovr_Lookup.Ptr(name, data);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_GetExternalCameras(
    ovrSession session,
    ovrExternalCamera* outCameras,
    unsigned int* outCameraCount) {
  if (!API.ovr_GetExternalCameras.Ptr)
    return ovrError_NotInitialized;
  if (!outCameras || !outCameraCount)
    return ovrError_InvalidParameter;

  return API.ovr_GetExternalCameras.Ptr(session, outCameras, outCameraCount);
}

OVR_PUBLIC_FUNCTION(ovrResult)
ovr_SetExternalCameraProperties(
    ovrSession session,
    const char* name,
    const ovrCameraIntrinsics* const intrinsics,
    const ovrCameraExtrinsics* const extrinsics) {
  if (!API.ovr_SetExternalCameraProperties.Ptr)
    return ovrError_NotInitialized;
  if (!name || (!intrinsics && !extrinsics))
    return ovrError_InvalidParameter;

  return API.ovr_SetExternalCameraProperties.Ptr(session, name, intrinsics, extrinsics);
}
