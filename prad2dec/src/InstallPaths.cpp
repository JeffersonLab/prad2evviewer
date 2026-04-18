// InstallPaths.cpp — implementation of the run-time data-directory resolver.
//
// See InstallPaths.h for the lookup policy.  Platform-specific bits:
//
//   Linux  :  dladdr() on our own symbol → the owning .so/exe path.
//             (readlink /proc/self/exe would give the Python interpreter
//             when called from the prad2py module, which is wrong.)
//   macOS  :  dladdr() again — identical story.
//   Windows:  GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS)
//             + GetModuleFileNameW.
//
// Needs C++17 <filesystem>.  Link ${CMAKE_DL_LIBS} on Linux (empty on
// glibc ≥ 2.34, -ldl on older systems) — handled by prad2dec's
// CMakeLists.
//=============================================================================

#include "InstallPaths.h"

#include <cstdlib>
#include <filesystem>
#include <system_error>

#if defined(_WIN32)
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #include <dlfcn.h>
#endif

namespace fs = std::filesystem;

namespace prad2 {

std::string module_dir()
{
#if defined(_WIN32)
    HMODULE h = nullptr;
    // Use the address of this function as the anchor — whichever DLL /
    // exe embeds the prad2dec static library will own it.
    if (!GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
            GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            reinterpret_cast<LPCWSTR>(&module_dir), &h)) {
        return {};
    }
    wchar_t buf[MAX_PATH];
    DWORD n = GetModuleFileNameW(h, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    std::wstring ws(buf, n);
    std::error_code ec;
    fs::path p = fs::weakly_canonical(fs::path(ws), ec);
    if (ec) p = fs::path(ws);
    return p.parent_path().string();
#else
    Dl_info info{};
    if (!dladdr(reinterpret_cast<const void *>(&module_dir), &info) ||
        !info.dli_fname) {
        return {};
    }
    std::error_code ec;
    fs::path p = fs::weakly_canonical(fs::path(info.dli_fname), ec);
    if (ec) p = fs::path(info.dli_fname);
    return p.parent_path().string();
#endif
}

std::string resolve_data_dir(const char *env_name,
                             std::initializer_list<const char *> rel_candidates,
                             const char *compile_default)
{
    if (env_name) {
        if (const char *env = std::getenv(env_name); env && *env) {
            return env;
        }
    }

    std::string base = module_dir();
    if (!base.empty()) {
        std::error_code ec;
        for (const char *rel : rel_candidates) {
            if (!rel) continue;
            fs::path cand = fs::path(base) / rel;
            fs::path norm = fs::weakly_canonical(cand, ec);
            if (ec) { ec.clear(); norm = cand; }
            if (fs::is_directory(norm, ec)) {
                return norm.string();
            }
            ec.clear();
        }
    }

    return compile_default ? std::string(compile_default) : std::string();
}

} // namespace prad2
