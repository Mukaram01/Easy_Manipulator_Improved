// CPU-only classification; observations here are synthetic, not GPU evidence.
#include "gl_context_check.hh"
#include <cassert>
int main() {
  Json::Value r;r["egl_query_available"]=r["glx_query_available"]=true;
  r["egl_current"]=r["glx_current"]=false;
  r["major"]=r["minor"]=0;r["query_error"]=r["prior_error"]=0;
  assert(CheckGlContext(r)=="NO_CURRENT_CONTEXT");
  r["egl_current"]=true;r["version"]="4.5 (Core Profile) Mesa";r["renderer"]="llvmpipe";
  r["major"]=4;r["minor"]=4;
  assert(CheckGlContext(r)=="CURRENT_CONTEXT_BELOW_GL45");
  r["minor"]=5;assert(CheckGlContext(r)=="PASS_CURRENT_GL45");
  r["query_error"]=1280;assert(CheckGlContext(r)=="INVALID_QUERY_OR_STATE");
  r["query_error"]=0;r["prior_error"]=1282;assert(CheckGlContext(r)=="INVALID_QUERY_OR_STATE");
  r["prior_error"]=0;r["version"]=Json::nullValue;assert(CheckGlContext(r)=="INVALID_QUERY_OR_STATE");
  r["version"]="4.5 Mesa";r["glx_query_available"]=false;
  assert(CheckGlContext(r)=="UNKNOWN_CONTEXT_API");
  r["glx_query_available"]=true;r["glx_current"]=true;
  assert(CheckGlContext(r)=="AMBIGUOUS_CURRENT_CONTEXT");
}
