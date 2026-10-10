#pragma once
#include <jsoncpp/json/json.h>
#include <string>
inline std::string CheckGlContext(const Json::Value &r) {
  if(!r["egl_query_available"].asBool() || !r["glx_query_available"].asBool())return "UNKNOWN_CONTEXT_API";
  if(!r["egl_current"].asBool() && !r["glx_current"].asBool())return "NO_CURRENT_CONTEXT";
  if(r["egl_current"].asBool() && r["glx_current"].asBool())return "AMBIGUOUS_CURRENT_CONTEXT";
  if(r["prior_error"].asUInt()!=0 || r["query_error"].asUInt()!=0 ||
     !r["version"].isString() || r["version"].asString().empty() ||
     !r["renderer"].isString() || r["renderer"].asString().empty() || r["major"].asInt()<1)
    return "INVALID_QUERY_OR_STATE";
  if(r["major"].asInt()<4 || (r["major"].asInt()==4 && r["minor"].asInt()<5))return "CURRENT_CONTEXT_BELOW_GL45";
  return "PASS_CURRENT_GL45";
}
