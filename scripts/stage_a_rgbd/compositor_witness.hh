#pragma once
// Opt-in post-execution observations only. No GPU/scene setters or new workspace.
#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>
#include <OgreSceneManager.h>
#include <OgreTextureGpu.h>
#include <OgrePixelFormatGpuUtils.h>
#include <OgreRenderPassDescriptor.h>
#include <Compositor/OgreCompositorNode.h>
#include <Compositor/OgreCompositorWorkspace.h>
#include <Compositor/OgreCompositorWorkspaceListener.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuad.h>
#include <sstream>
#include <map>

inline std::string processToken(const void *p) {std::ostringstream s;s<<p;return s.str();}
inline Json::Value finiteScalar(float v) {
  if(std::isfinite(v))return static_cast<double>(v);
  return std::isnan(v)?"NaN":(v>0?"+INF":"-INF");
}
inline Json::Value textureResource(Ogre::TextureGpu *t) {
  if(!t)return Json::Value();
  Json::Value r;r["name"]=t->getNameStr();r["identity"]=processToken(t);
  r["width"]=t->getWidth();r["height"]=t->getHeight();
  r["format"]=Ogre::PixelFormatGpuUtils::toString(t->getPixelFormat());
  return r;
}
inline Json::Value executingPassResource(Ogre::Pass *pass) {
  Json::Value r;if(!pass || !pass->hasFragmentProgram())return r;
  auto program=pass->getFragmentProgram()->_getBindingDelegate();
  r["fragment_program"]=program->getName();r["source_file"]=program->getSourceFile();
  const auto parameters=pass->getFragmentProgramParameters();
  for(const auto &name:{"near","far","projectionParams"}) {
    auto definition=parameters->_findNamedConstantDefinition(name,false);if(!definition)continue;
    if(!readableFloatUniform(*definition,std::string(name)=="projectionParams"?2u:1u,parameters->getFloatConstantList().size())) {r["uniform_status"][name]="UNKNOWN_UNSUPPORTED_STORAGE";continue;}
    const auto data=static_cast<const Ogre::GpuProgramParameters&>(*parameters).getFloatPointer(definition->physicalIndex);
    Json::Value values(Json::arrayValue);
    for(unsigned i=0;i<(std::string(name)=="projectionParams"?2u:1u);++i)values.append(finiteScalar(data[i]));
    r["uniforms"][name]=values;
  }
  r["scope"]="executing quad's actual Ogre Pass resource; not GPU program correspondence";
  return r;
}

struct CompositorWitness final:Ogre::CompositorWorkspaceListener {
  Json::Value &report;int &batch;int &camera;std::filesystem::path output;
  std::set<Ogre::CompositorWorkspace*> workspaces;
  std::map<GLuint,std::string> binaries;
  CompositorWitness(Json::Value &r,int &b,int &c,const std::filesystem::path &o):report(r),batch(b),camera(c),output(o) {
    report["compositor_events"]=Json::Value(Json::arrayValue);
  }
  ~CompositorWitness(){for(auto workspace:workspaces)workspace->removeListener(this);}
  void observe(Ogre::Camera *c) {
    auto pass=c->getSceneManager()->getCurrentCompositorPass();if(!pass)return;
    // Public const view refers to the live mutable workspace created by Ogre.
    // Remove constness ONLY to subscribe/unsubscribe via its public listener API.
    auto workspace=const_cast<Ogre::CompositorWorkspace*>(pass->getParentNode()->getWorkspace());
    if(workspaces.insert(workspace).second)workspace->addListener(this);
  }
  Json::Value gpu() {
    Json::Value r;r["draw_time_binding"]="UNKNOWN";
    r["texture_unit_mapping"]="UNKNOWN_NONACTIVE_UNITS_NOT_QUERIED";
    r["executable_provenance"]="UNKNOWN";
    auto version=glGetString(GL_VERSION);
    if(!version){r["status"]="NO_CURRENT_GL_CONTEXT";return r;}
    r["version"]=reinterpret_cast<const char*>(version);
    GLint major=0,minor=0;glGetIntegerv(GL_MAJOR_VERSION,&major);glGetIntegerv(GL_MINOR_VERSION,&minor);
    if(major<4 || (major==4 && minor<5)){r["status"]="UNSUPPORTED_GL";return r;}
    r["status"]="POST_EXECUTION_STATE";r["gl_error_before"]=glGetError();
    auto integer=[](GLenum key){GLint v=0;glGetIntegerv(key,&v);return v;};
    GLint viewport[4];glGetIntegerv(GL_VIEWPORT,viewport);r["viewport"]=Json::Value(Json::arrayValue);
    for(auto v:viewport)r["viewport"].append(v);
    r["samples"]=integer(GL_SAMPLES);r["sample_buffers"]=integer(GL_SAMPLE_BUFFERS);
    r["subpixel_bits"]=integer(GL_SUBPIXEL_BITS);r["depth_func"]=integer(GL_DEPTH_FUNC);
    r["clip_origin"]=integer(GL_CLIP_ORIGIN);r["clip_depth_mode"]=integer(GL_CLIP_DEPTH_MODE);
    r["sample_mask_enabled"]=static_cast<bool>(glIsEnabled(GL_SAMPLE_MASK));
    r["sample_coverage_enabled"]=static_cast<bool>(glIsEnabled(GL_SAMPLE_COVERAGE));
    r["polygon_offset_fill"]=static_cast<bool>(glIsEnabled(GL_POLYGON_OFFSET_FILL));
    GLboolean writeMask=GL_FALSE;glGetBooleanv(GL_DEPTH_WRITEMASK,&writeMask);r["depth_write_mask"]=static_cast<bool>(writeMask);
    r["clip_distances"]=Json::Value(Json::arrayValue);for(int i=0;i<8;++i)r["clip_distances"].append(static_cast<bool>(glIsEnabled(GL_CLIP_DISTANCE0+i)));
    r["multisample"]=static_cast<bool>(glIsEnabled(GL_MULTISAMPLE));
    r["depth_test"]=static_cast<bool>(glIsEnabled(GL_DEPTH_TEST));
    r["depth_clamp"]=static_cast<bool>(glIsEnabled(GL_DEPTH_CLAMP));
    r["scissor_test"]=static_cast<bool>(glIsEnabled(GL_SCISSOR_TEST));
    GLint scissor[4];glGetIntegerv(GL_SCISSOR_BOX,scissor);r["scissor_box"]=Json::Value(Json::arrayValue);
    for(auto v:scissor)r["scissor_box"].append(v);
    GLdouble range[2];glGetDoublev(GL_DEPTH_RANGE,range);r["depth_range"]=Json::Value(Json::arrayValue);
    for(auto v:range)r["depth_range"].append(v);
    auto fbo=integer(GL_DRAW_FRAMEBUFFER_BINDING);r["draw_framebuffer"]=fbo;
    if(fbo)for(auto attachment:{GL_DEPTH_ATTACHMENT,GL_COLOR_ATTACHMENT0}) {
      GLint type=0,name=0;glGetNamedFramebufferAttachmentParameteriv(fbo,attachment,GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,&type);
      Json::Value a;a["object_type"]=type;
      if(type!=GL_NONE)glGetNamedFramebufferAttachmentParameteriv(fbo,attachment,GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,&name);
      a["object_name"]=name;
      if(type==GL_TEXTURE && name) {
        GLint level=0;glGetNamedFramebufferAttachmentParameteriv(fbo,attachment,GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL,&level);
        GLint w=0,h=0,format=0;glGetTextureLevelParameteriv(name,level,GL_TEXTURE_WIDTH,&w);
        glGetTextureLevelParameteriv(name,level,GL_TEXTURE_HEIGHT,&h);glGetTextureLevelParameteriv(name,level,GL_TEXTURE_INTERNAL_FORMAT,&format);
        a["width"]=w;a["height"]=h;a["format"]=format;a["level"]=level;
      } else a["format"]=Json::Value();
      r[attachment==GL_DEPTH_ATTACHMENT?"depth_attachment":"colour_attachment"]=a;
    }
    GLint pipeline=integer(GL_PROGRAM_PIPELINE_BINDING),program=integer(GL_CURRENT_PROGRAM),fragment=0;
    // With GL_CURRENT_PROGRAM this handle is monolithic; stage inventory is
    // unknown. The legacy fragment_program key is NOT fragment-stage proof.
    if(program)fragment=program;
    else if(pipeline)glGetProgramPipelineiv(pipeline,GL_FRAGMENT_SHADER,&fragment);
    r["pipeline"]=pipeline;r["current_program"]=program;r["fragment_program"]=fragment;
    if(fragment) {
      GLint linked=0;glGetProgramiv(fragment,GL_LINK_STATUS,&linked);r["fragment_linked"]=linked;
      GLint count=0,maxName=0;glGetProgramiv(fragment,GL_ACTIVE_UNIFORMS,&count);glGetProgramiv(fragment,GL_ACTIVE_UNIFORM_MAX_LENGTH,&maxName);
      r["active_uniform_count"]=count;
      if(count>=0 && count<1024 && maxName>0 && maxName<65536)for(GLint i=0;i<count;++i) {
        std::vector<GLchar> name(maxName);GLsizei length=0;GLint size=0;GLenum type=0;
        glGetActiveUniform(fragment,i,maxName,&length,&size,&type,name.data());
        std::string key(name.data(),length);GLint location=glGetUniformLocation(fragment,key.c_str());
        r["uniform_metadata"][key]["type"]=type;r["uniform_metadata"][key]["location"]=location;
        if(location<0 || size!=1){r["uniform_metadata"][key]["value_status"]="UNKNOWN_BLOCK_OR_ARRAY";continue;}
        Json::Value values(Json::arrayValue);
        unsigned n=type==GL_FLOAT?1:type==GL_FLOAT_VEC2?2:type==GL_FLOAT_VEC3?3:type==GL_FLOAT_VEC4?4:0;
        if(n){GLfloat v[4]={};glGetUniformfv(fragment,location,v);for(unsigned j=0;j<n;++j)values.append(finiteScalar(v[j]));}
        else if(type==GL_INT || type==GL_SAMPLER_2D || type==GL_UNSIGNED_INT_SAMPLER_2D) {
          GLint v=0;glGetUniformiv(fragment,location,&v);values.append(v);
          if(type!=GL_INT && v>=0 && v<integer(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS)) {
            GLint sampler=0;glGetIntegeri_v(GL_SAMPLER_BINDING,v,&sampler);
            r["samplers"][key]["unit"]=v;r["samplers"][key]["object"]=sampler;
            if(sampler)for(auto param:{GL_TEXTURE_MIN_FILTER,GL_TEXTURE_MAG_FILTER,GL_TEXTURE_WRAP_S,GL_TEXTURE_WRAP_T,GL_TEXTURE_COMPARE_MODE}) {
              GLint value=0;glGetSamplerParameteriv(sampler,param,&value);r["samplers"][key]["parameters"][std::to_string(param)]=value;
            }
          }
        } else {r["uniform_metadata"][key]["value_status"]="UNKNOWN_TYPE";continue;}
        r["uniforms"][key]=values;
      }
      GLint bytes=0;glGetProgramiv(fragment,GL_PROGRAM_BINARY_LENGTH,&bytes);
      r["binary_length"]=bytes;
      if(bytes>0 && bytes<16*1024*1024) {
        if(!binaries.count(fragment)) {
          std::vector<unsigned char> binary(bytes);GLsizei length=0;GLenum format=0;
          glGetProgramBinary(fragment,bytes,&length,&format,binary.data());
          auto path=output.string()+".program-"+std::to_string(fragment)+".bin";
          std::ofstream stream(path,std::ios::binary);stream.write(reinterpret_cast<const char*>(binary.data()),length);
          if(stream && length>0)binaries[fragment]=path;
          r["binary_format"]=format;
        }
        if(binaries.count(fragment)){r["binary_path"]=binaries[fragment];r["executable_provenance"]="POST_PASS_GL_PROGRAM_BINARY";}
      }
    }
    r["active_texture_unit"]=integer(GL_ACTIVE_TEXTURE)-GL_TEXTURE0;
    r["active_unit_texture_2d"]=integer(GL_TEXTURE_BINDING_2D);
    r["gl_error_after"]=glGetError();return r;
  }
  void passPosExecute(Ogre::CompositorPass *pass) override {
    Json::Value e;e["session"]=report["native_session_id"];e["batch"]=batch;e["camera"]=camera;
    e["sequence"]=report["compositor_events"].size();e["phase"]="pass_post";
    e["acquisition_sequence"]=report["acquisition_events"].size();e["pass_id"]=processToken(pass);
    e["workspace_id"]=processToken(pass->getParentNode()->getWorkspace());
    e["pass_type"]=pass->getType()==Ogre::PASS_QUAD?"quad":pass->getType()==Ogre::PASS_SCENE?"scene":"other";
    auto target=pass->getRenderPassDesc();
    if(target){e["target_descriptor"]["depth"]=textureResource(target->mDepth.texture);e["target_descriptor"]["colour0"]=textureResource(target->mColour[0].texture);}
    if(auto quad=dynamic_cast<Ogre::CompositorPassQuad*>(pass)) {
      e["pass_resource"]=executingPassResource(quad->getPass());
      if(quad->getCamera())e["quad_camera_id"]=Json::UInt64(quad->getCamera()->getId());
    }
    e["gl"]=gpu();report["compositor_events"].append(e);
  }
};
