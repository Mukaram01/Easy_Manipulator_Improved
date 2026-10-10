// Opt-in same-fragment RGB/uint-ID fixture; no Gazebo, EPD, physics or authority.
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
    ++calls;GLint fbo=0;glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING,&fbo);
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

int main(int argc,char **argv){
  if(argc!=2)return 2;
  for(const auto &suffix:{"",".rgb8",".ids.u32"})
    if(std::filesystem::exists(std::string(argv[1])+suffix))return 2;
  static_assert(sizeof(unsigned)==4,"uint32 readback required");
  const std::string path=argv[1];Json::Value r;r["contact_authority"]=false;r["execution_goals"]=0;
  r["session"]=std::to_string(getpid())+":"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  r["schema"]="workcell_same_fragment_fixture/v1";r["decision"]="BLOCKED";
  bool ok=false;
  try{
    auto engine=ignition::rendering::engine("ogre2");if(!engine)throw std::runtime_error("missing Ogre2");
    GLint major=0,minor=0;glGetIntegerv(GL_MAJOR_VERSION,&major);glGetIntegerv(GL_MINOR_VERSION,&minor);
    if(major<4||(major==4&&minor<5))throw std::runtime_error("GL4.5 required for typed clear/attachment validation");
    auto scene=engine->CreateScene("same_fragment_only");
    auto native=std::dynamic_pointer_cast<ignition::rendering::Ogre2Scene>(scene);
    if(!native)throw std::runtime_error("missing native scene");auto sm=native->OgreSceneManager();
    auto camera=sm->createCamera("same_fragment_camera");camera->setNearClipDistance(.1);camera->setFarClipDistance(10);
    camera->setAspectRatio(1);camera->setFOVy(Ogre::Radian(1.0471975511965976));
    // Ogre 2.2 createCamera already attaches to the dynamic root (public contract).
    if(camera->getParentSceneNode()!=sm->getRootSceneNode(Ogre::SCENE_DYNAMIC))
      throw std::runtime_error("unexpected camera parent");
    auto &pm=Ogre::HighLevelGpuProgramManager::getSingleton();
    for(auto stage:{Ogre::GPT_VERTEX_PROGRAM,Ogre::GPT_FRAGMENT_PROGRAM}){
      auto p=pm.createProgram(stage==Ogre::GPT_VERTEX_PROGRAM?"fragment_vs":"fragment_fs","General","glsl",stage);
      p->setSource(stage==Ogre::GPT_VERTEX_PROGRAM?vs:fs);p->load();if(p->hasCompileError())throw std::runtime_error("shader compile error");
    }
    r["vertex_shader"]=vs;r["fragment_shader"]=fs;
    // Near cube is submitted first; farther cube overlaps it but must not replace its pixels.
    const unsigned ids[2]={16777217u,4000000022u};
    for(int i=0;i<2;++i){
      auto name="fragment_cube_"+std::to_string(i);auto mat=Ogre::MaterialManager::getSingleton().create(name,"General");
      auto pass=mat->getTechnique(0)->getPass(0);pass->setVertexProgram("fragment_vs");pass->setFragmentProgram("fragment_fs");
      pass->getVertexProgramParameters()->setNamedAutoConstant("worldViewProj",Ogre::GpuProgramParameters::ACT_WORLDVIEWPROJ_MATRIX);
      pass->getFragmentProgramParameters()->setNamedConstant("baseColour",i?Ogre::Vector3(0,1,0):Ogre::Vector3(1,0,0));
      pass->getFragmentProgramParameters()->setNamedConstant("entityId",ids[i]);
      Ogre::HlmsMacroblock macro;macro.mDepthCheck=true;macro.mDepthWrite=true;
      pass->setMacroblock(macro);Ogre::HlmsBlendblock blend;
      blend.mSourceBlendFactor=Ogre::SBF_ONE;blend.mDestBlendFactor=Ogre::SBF_ZERO;
      pass->setBlendblock(blend);
      mat->load();auto entity=sm->createEntity(Ogre::SceneManager::PT_CUBE);entity->setMaterialName(name);
      auto node=sm->getRootSceneNode()->createChildSceneNode();node->setScale(.01,.01,.01);
      node->setPosition(i?.4:-.2,0,i?-3.0:-2.0);node->attachObject(entity);
      Json::Value v;v["id"]=ids[i];v["native_entity"]=std::to_string(entity->getId());v["name"]=name;
      v["material"]=name;v["relationship"]="native fixture only; no Gazebo collision mapping";
      r["inventory"].append(v);
    }
    auto actualEntities=sm->getMovableObjectIterator("Entity");unsigned actualCount=0;
    while(actualEntities.hasMoreElements()){auto e=actualEntities.getNext();++actualCount;
      bool found=false;for(const auto &v:r["inventory"])found|=v["native_entity"].asString()==std::to_string(e->getId());
      if(!found)throw std::runtime_error("untracked fixture entity");
    }
    if(actualCount!=2)throw std::runtime_error("incomplete fixture entity inventory");
    r["inventory_complete"]=true;r["camera_id"]=Json::UInt64(camera->getId());
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
    workspace->_beginUpdate(true);workspace->_update();workspace->_endUpdate(true);workspace->removeListener(&formats);
    if(formats.calls!=1)throw std::runtime_error("incomplete scene-pass inventory");
    std::vector<unsigned char> rgba(256*256*4),rgb(256*256*3);std::vector<unsigned> labels(256*256);
    for(int i=0;i<2;++i){auto src=textures[i]->getEmptyBox(0);Ogre::TextureBox dst(256,256,1,1,4,256*4,256*256*4);
      dst.data=i?static_cast<void*>(labels.data()):rgba.data();textures[i]->copyContentsToMemory(src,dst,i?Ogre::PFG_R32_UINT:Ogre::PFG_RGBA8_UNORM,false);
    }
    scene->PostRender();
    std::set<unsigned> visible;unsigned counts[2]={};
    for(size_t p=0;p<labels.size();++p){for(int c=0;c<3;++c)rgb[p*3+c]=rgba[p*4+c];visible.insert(labels[p]);
      if(labels[p]==ids[0])++counts[0];if(labels[p]==ids[1])++counts[1];
    }
    r["center_id"]=labels[128*256+128];r["visible_counts"].append(counts[0]);r["visible_counts"].append(counts[1]);
    ok=visible==std::set<unsigned>{0,ids[0],ids[1]}&&counts[0]>0&&counts[1]>0&&labels[128*256+128]==ids[0];
    auto save=[&](std::string suffix,const void* data,size_t n){std::ofstream out(path+suffix,std::ios::binary);out.write(static_cast<const char*>(data),n);if(!out)throw std::runtime_error("write failed");};
    save(".rgb8",rgb.data(),rgb.size());save(".ids.u32",labels.data(),labels.size()*4);
    r["width"]=r["height"]=256;r["frame"]=1;r["shared_scene_pass"]=true;r["samples"]=1;
    r["opaque"]=true;r["blending"]=false;r["depth_test"]=true;r["depth_write"]=true;
    r["rgb_format"]="RGBA8_UNORM";r["id_format"]="R32_UINT";r["background_id"]=0;r["invalid_id"]=Json::UInt(4294967295u);
    r["fixture_result"]=ok?"PASS":"BLOCKED";r["physical_mapping"]="BLOCKED_NO_GAZEBO_INVENTORY";
    r["driver"]=reinterpret_cast<const char*>(glGetString(GL_RENDERER));r["gl_version"]=reinterpret_cast<const char*>(glGetString(GL_VERSION));
    cm->removeWorkspace(workspace);engine->DestroyScene(scene);
  }catch(const std::exception &e){r["failure_reason"]=e.what();ok=false;}
  std::ifstream maps("/proc/self/maps");std::string line;std::set<std::string> loaded;
  while(std::getline(maps,line)){auto p=line.find('/');if(p!=std::string::npos&&line.find(".so",p)!=std::string::npos)loaded.insert(line.substr(p));}
  for(const auto &p:loaded)r["loaded_library_paths"].append(p);
  std::ofstream output(path);output<<r<<'\n';return ok&&output?0:2;
}
