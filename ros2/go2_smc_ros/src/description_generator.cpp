#include <mujoco/mujoco.h>
#include <tinyxml2.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

std::string attribute(const tinyxml2::XMLElement* element, const char* key, const char* fallback = "") {
  const char* value = element->Attribute(key);
  return value ? value : fallback;
}
std::string rpy(const std::string& text) {
  double w=1, x=0, y=0, z=0;
  if (std::sscanf(text.c_str(), "%lf %lf %lf %lf", &w, &x, &y, &z) != 4) throw std::runtime_error("Invalid quaternion");
  const double n=std::sqrt(w*w+x*x+y*y+z*z); w/=n; x/=n; y/=n; z/=n;
  return std::to_string(std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)))+" "+
      std::to_string(std::asin(std::clamp(2*(w*y-z*x),-1.0,1.0)))+" "+
      std::to_string(std::atan2(2*(w*z+x*y),1-2*(y*y+z*z)));
}
void body(const tinyxml2::XMLElement* element, const std::string& parent, const mjModel* model, std::ostream& out) {
  const std::string raw = attribute(element, "name");
  const std::string name = raw == "base" ? "base_link" : raw;
  out << "<link name=\"" << name << "\">\n";
  for (auto* geom=element->FirstChildElement("geom"); geom; geom=geom->NextSiblingElement("geom")) {
    if (attribute(geom,"class") != "visual") continue;
    const std::string mesh=attribute(geom,"mesh");
    const int material=mj_name2id(model,mjOBJ_MATERIAL,attribute(geom,"material").c_str());
    out << "<visual><origin xyz=\"" << attribute(geom,"pos","0 0 0") << "\" rpy=\""
        << rpy(attribute(geom,"quat","1 0 0 0")) << "\"/><geometry><mesh filename=\"package://go2_smc_ros/meshes/"
        << mesh << ".obj\"/></geometry><material name=\"" << name << '_' << mesh << "\"><color rgba=\"";
    for (int k=0;k<4;++k) out << (material>=0 ? model->mat_rgba[4*material+k] : 1.0) << (k==3 ? "" : " ");
    out << "\"/></material></visual>\n";
  }
  out << "</link>\n";
  if (!parent.empty()) {
    const auto* joint=element->FirstChildElement("joint");
    if (!joint) throw std::runtime_error("Expected hinge joint in " + raw);
    const std::string joint_name=attribute(joint,"name");
    const int id=mj_name2id(model,mjOBJ_JOINT,joint_name.c_str());
    if (id<0) throw std::runtime_error("Unknown joint");
    out << "<joint name=\""<<joint_name<<"\" type=\"revolute\"><parent link=\""<<parent<<"\"/><child link=\""<<name
        <<"\"/><origin xyz=\""<<attribute(element,"pos","0 0 0")<<"\" rpy=\""<<rpy(attribute(element,"quat","1 0 0 0"))
        <<"\"/><axis xyz=\""<<model->jnt_axis[3*id]<<' '<<model->jnt_axis[3*id+1]<<' '<<model->jnt_axis[3*id+2]
        <<"\"/><limit lower=\""<<model->jnt_range[2*id]<<"\" upper=\""<<model->jnt_range[2*id+1]
        <<"\" effort=\"45.43\" velocity=\"30\"/></joint>\n";
  }
  for (auto* child=element->FirstChildElement("body");child;child=child->NextSiblingElement("body")) body(child,name,model,out);
}
int main(int argc,char** argv) {
  try {
    if(argc!=3) throw std::runtime_error("description_generator go2.xml output.urdf");
    char error[2048]{};
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(argv[1],nullptr,error,sizeof(error)),mj_deleteModel);
    if(!model) throw std::runtime_error(error);
    tinyxml2::XMLDocument doc;
    if(doc.LoadFile(argv[1])!=tinyxml2::XML_SUCCESS) throw std::runtime_error("Cannot read MJCF");
    std::ofstream out(argv[2]); if(!out) throw std::runtime_error("Cannot write URDF");
    out << "<?xml version=\"1.0\"?>\n<!-- Generated from the original Menagerie Go2 MJCF and OBJ meshes. RViz display only. -->\n<robot name=\"unitree_go2\">\n";
    body(doc.FirstChildElement("mujoco")->FirstChildElement("worldbody")->FirstChildElement("body"),"",model.get(),out);
    out << "</robot>\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
