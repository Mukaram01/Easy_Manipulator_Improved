#pragma once
// Shared opt-in same-fragment compositor; no contact or timing authority.
#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/ogre2/Ogre2Scene.hh>
#include <OgreRoot.h>
#include <OgreRenderSystem.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreHlmsDatablock.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureGpu.h>
#include <OgreTextureBox.h>
#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/OgreCompositorWorkspaceDef.h>
#include <Compositor/OgreCompositorWorkspace.h>
#include <Compositor/OgreCompositorWorkspaceListener.h>
#include <Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <chrono>
#include <unistd.h>

const char *vs=R"(#version 330 core
in vec4 vertex;
in vec3 normal;
uniform mat4 worldViewProj;
out vec3 faceNormal;
void main(){ gl_Position=worldViewProj*vertex;faceNormal=normal; }
)";
const char *fs=R"(#version 330 core
in vec3 faceNormal;
uniform vec3 baseColour;
uniform uint entityId;
layout(location=0) out vec4 rgb;
layout(location=1) out uint identity;
void main(){ rgb=vec4(baseColour*(0.35+0.65*abs(normalize(faceNormal).z)),1.0);identity=entityId; }
)";

struct Formats:Ogre::CompositorWorkspaceListener {
  Json::Value &r; explicit Formats(Json::Value &v):r(v){}
  void passPosExecute(Ogre::CompositorPass*) override {
    ++calls;r["render_pass_count"]=calls;
    r["post_pass_depth_test"]=static_cast<bool>(glIsEnabled(GL_DEPTH_TEST));
    GLboolean depthWrite=GL_FALSE;glGetBooleanv(GL_DEPTH_WRITEMASK,&depthWrite);
    r["post_pass_depth_write"]=static_cast<bool>(depthWrite);
    GLint fbo=0;glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING,&fbo);
    for(int i=0;i<2;++i){GLint type=0,name=0,format=0,w=0,h=0;
      glGetNamedFramebufferAttachmentParameteriv(fbo,GL_COLOR_ATTACHMENT0+i,GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,&type);
      if(type!=GL_TEXTURE)throw std::runtime_error("attachment is not texture");
      glGetNamedFramebufferAttachmentParameteriv(fbo,GL_COLOR_ATTACHMENT0+i,GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,&name);
      glGetTextureLevelParameteriv(name,0,GL_TEXTURE_INTERNAL_FORMAT,&format);
      glGetTextureLevelParameteriv(name,0,GL_TEXTURE_WIDTH,&w);glGetTextureLevelParameteriv(name,0,GL_TEXTURE_HEIGHT,&h);
      Json::Value a;a["texture"]=name;a["format"]=format;a["width"]=w;a["height"]=h;r["actual_attachments"].append(a);
      if(format!=(i?GL_R32UI:GL_RGBA8)||w!=256||h!=256)throw std::runtime_error("unexpected attachment format/size");
    }
    GLint samples=0;glGetIntegerv(GL_SAMPLES,&samples);if(samples)throw std::runtime_error("unexpected MSAA");
    if(glGetError()!=GL_NO_ERROR)throw std::runtime_error("GL attachment query error");
  }
  int calls=0;
};


inline std::vector<unsigned> CaptureFragmentMrt(const ignition::rendering::ScenePtr &scene, Ogre::SceneManager *sm, Ogre::Camera *camera, Json::Value &r, const std::string &path) {
    auto root=Ogre::Root::getSingletonPtr();auto rs=root->getRenderSystem();auto tm=rs->getTextureGpuManager();
    Ogre::CompositorChannelVec textures;
    for(int i=0;i<2;++i){auto t=tm->createTexture("fragment_output_"+std::to_string(i),Ogre::GpuPageOutStrategy::Discard,
      Ogre::TextureFlags::RenderToTexture,Ogre::TextureTypes::Type2D);
      t->setResolution(256,256);t->setPixelFormat(i?Ogre::PFG_R32_UINT:Ogre::PFG_RGBA8_UNORM);
      t->scheduleTransitionTo(Ogre::GpuResidency::Resident);textures.push_back(t);
    }
    auto cm=root->getCompositorManager2();auto nd=cm->addNodeDefinition("fragment_node");
    for(int i=0;i<2;++i)nd->addTextureSourceName(i?"ids":"rgb",i,Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    auto view=nd->addRenderTextureView("fragment_mrt");view->colourAttachments.resize(2);
    view->colourAttachments[0].textureName="rgb";view->colourAttachments[1].textureName="ids";
    view->depthBufferFormat=Ogre::PFG_D32_FLOAT;view->preferDepthTexture=true;
    nd->setNumTargetPass(1);
    auto target=nd->addTargetPass("fragment_mrt");
    auto sp=static_cast<Ogre::CompositorPassSceneDef*>(target->addPass(Ogre::PASS_SCENE));
    // Ogre2.2's colour clear uses float clear calls: avoid those for uint attachment.
    // Initialize owned textures with the GL-defined typed clear before the sole scene pass.
    sp->mLoadActionColour[0]=sp->mLoadActionColour[1]=Ogre::LoadAction::Load;
    sp->mStoreActionColour[0]=sp->mStoreActionColour[1]=Ogre::StoreAction::Store;
    sp->mLoadActionDepth=Ogre::LoadAction::Clear;sp->mStoreActionDepth=Ogre::StoreAction::Store;
    sp->mEnableForwardPlus=false;
    auto wd=cm->addWorkspaceDefinition("fragment_workspace");wd->connectExternal(0,"fragment_node",0);wd->connectExternal(1,"fragment_node",1);
    auto workspace=cm->addWorkspace(sm,textures,camera,"fragment_workspace",false);
    scene->PreRender();
    const GLuint zero=0;const unsigned char black[4]={0,0,0,255};
    for(int i=0;i<2;++i){GLuint id=0;textures[i]->getCustomAttribute(Ogre::TextureGpu::msFinalTextureBuffer,&id);
      if(!id)throw std::runtime_error("missing GL texture");
      glClearTexImage(id,0,i?GL_RED_INTEGER:GL_RGBA,i?GL_UNSIGNED_INT:GL_UNSIGNED_BYTE,i?static_cast<const void*>(&zero):black);
    }
    if(glGetError()!=GL_NO_ERROR)throw std::runtime_error("typed texture clear failed");
    Formats formats(r);workspace->addListener(&formats);
    sm->updateSceneGraph();
    workspace->_beginUpdate(true);workspace->_update();workspace->_endUpdate(true);workspace->removeListener(&formats);
    if(formats.calls!=1)throw std::runtime_error("incomplete scene-pass inventory");
    std::vector<unsigned char> rgba(256*256*4),rgb(256*256*3);std::vector<unsigned> labels(256*256);
    for(int i=0;i<2;++i){auto src=textures[i]->getEmptyBox(0);Ogre::TextureBox dst(256,256,1,1,4,256*4,256*256*4);
      dst.data=i?static_cast<void*>(labels.data()):rgba.data();textures[i]->copyContentsToMemory(src,dst,i?Ogre::PFG_R32_UINT:Ogre::PFG_RGBA8_UNORM,false);
    }
    scene->PostRender();
    for(size_t p=0;p<labels.size();++p)for(int c=0;c<3;++c)rgb[p*3+c]=rgba[p*4+c];
    auto save=[&](std::string suffix,const void* data,size_t n){std::ofstream out(path+suffix,std::ios::binary);out.write(static_cast<const char*>(data),n);if(!out)throw std::runtime_error("write failed");};
    save(".rgb8",rgb.data(),rgb.size());save(".ids.u32",labels.data(),labels.size()*4);
    r["width"]=r["height"]=256;r["frame"]=1;r["shared_scene_pass"]=true;r["samples"]=1;
    r["opaque"]=true;r["blending"]=false;r["depth_test"]=true;r["depth_write"]=true;
    r["rgb_format"]="RGBA8_UNORM";r["id_format"]="R32_UINT";r["background_id"]=0;r["invalid_id"]=Json::UInt(4294967295u);
    r["driver"]=reinterpret_cast<const char*>(glGetString(GL_RENDERER));r["gl_version"]=reinterpret_cast<const char*>(glGetString(GL_VERSION));
    cm->removeWorkspace(workspace);return labels;
}
