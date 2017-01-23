#ifndef XML_EXPORTER_H
#define XML_EXPORTER_H

#include <string>

class RigidBody3DState;

namespace XMLExporter
{
  bool saveToXMLFile( const std::string& file_name, const RigidBody3DState& state );
}

#endif
