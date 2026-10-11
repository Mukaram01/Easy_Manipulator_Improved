#pragma once
#include "gl_context_check.hh"
#include <dlfcn.h>
#include <sstream>
#include <thread>
#include <sys/syscall.h>
#include <unistd.h>
// Query only public EGL/GLX entry points already loaded in this process.
// No context creation, private structures, or cross-thread context transfer.
inline Json::Value GlContextWitness() {
  Json::Value r;
  using CurrentContext=void *(*)();
  // Resolve EGL explicitly: Ogre may load it outside RTLD_DEFAULT.
  // Loading the library does not create or activate an EGL context.
  static void *eglLibrary=dlopen("libEGL.so.1",RTLD_NOW|RTLD_LOCAL);
  const auto egl=reinterpret_cast<CurrentContext>(
      eglLibrary?dlsym(eglLibrary,"eglGetCurrentContext"):nullptr);
  const auto glx=reinterpret_cast<CurrentContext>(dlsym(RTLD_DEFAULT,"glXGetCurrentContext"));
  r["egl_query_available"]=static_cast<bool>(egl);r["glx_query_available"]=static_cast<bool>(glx);
  auto witness=[&](const char *key,CurrentContext query) {
    const auto context=query?query():nullptr;std::ostringstream value;value<<context;
    r[std::string(key)+"_current"]=context!=nullptr;
    r[std::string(key)+"_context"]=query?Json::Value(value.str()):Json::Value();
  };
  witness("egl",egl);witness("glx",glx);
  std::ostringstream thread;thread<<std::this_thread::get_id();
  r["thread"]=thread.str();r["linux_tid"]=Json::Int64(syscall(SYS_gettid));
  // Preserve any existing error separately; never hide it in a successful gate.
  r["prior_error"]=Json::UInt(glGetError());
  const auto version=glGetString(GL_VERSION),renderer=glGetString(GL_RENDERER);
  r["version"]=version?Json::Value(reinterpret_cast<const char*>(version)):Json::Value();
  r["renderer"]=renderer?Json::Value(reinterpret_cast<const char*>(renderer)):Json::Value();
  GLint major=0,minor=0;glGetIntegerv(GL_MAJOR_VERSION,&major);glGetIntegerv(GL_MINOR_VERSION,&minor);
  r["major"]=major;r["minor"]=minor;r["query_error"]=Json::UInt(glGetError());
  r["classification"]=CheckGlContext(r);return r;
}
