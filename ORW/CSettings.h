#pragma once
#include <gl/glew.h>
#include <GLFW/glfw3.h>
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

//
// Handling Settings Events.
//
// Program Start:		Each subsystem must register its parameters.  bModified <= FALSE.
//		1. main calls CSettings::Clear()
//		2. main calls RegisterSettings on each subsystem
//		3. RegisterSettings calls SetValue for each parameter
//
// Parameter Change:	Each subsystem registers each changed parameter.  bModified <= TRUE
//		1. Subsystem calls SetValue for each changed parameter
//
// Save:				Tree is written.  bModified <= FALSE
//		1. main calls CSettings::Save
//		2. CSettings::Save calls pt::write_xml
//
// Open:				If bModified == TRUE, user is prompted to save.
//						Tree is read.  Each subsystem is informed.  Subsystem reads its Settings.  bModified <= FALSE
//		1. main calls CSettings::Load
//		1. Load calls SaveIfNeeded to prompt user and possibly save existing settings.  Note: may need to get filename from user.
//		2. Load clears tree via pt::erase at root.  Then calls pt::read_xml
//		3. main calls ImportSettings on each subsystem
//		4. ImportSettings calls CSettings::GetValue for each of its parameters
//
// New					If bModified == TRUE, user is prompted to save.  bModified <= FALSE
//		1. main calls CSettings::New
//		2. New calls SaveIfNeeded.
//		3. New clears tree via pt::erase.
//		4. main calls Reset on each subsystem
//		5. Reset resets all parameters to default
//		6. main calls RegisterSettings on each subsystem
//		3. RegisterSettings calls SetValue for each parameter 
//



// Exit:				If bModified == TRUE, user is prompted to save.
//		1. CSettings::~CSettings does this


namespace pt = boost::property_tree;

class CSettings
{
public:
	CSettings();
	~CSettings();

	template<class T> T GetValue(const std::string& group, const std::string& name, const T& defaultValue)
	{
		return m_Tree.get(ReplaceSpaces(group + "." + name), defaultValue);
	}
	template<class T> void SetValue(const std::string& group, const std::string& name, const T& value)
	{
		m_bModified = true;
		auto x = m_Tree.put(ReplaceSpaces(group + "." + name), value);
		//T q = GetValue<T>(group, name, (T) 0);
		//assert(q == value);
	}

	void New();
	bool Load(const std::string& filename);
	bool Save();
	void SaveAs();
	void Save(const std::string& filename);
	void SaveIfNeeded();
	void Clear(const std::string& group);
	
protected:
	pt::ptree m_Tree;

	bool m_bModified = false;
	std::string m_Filename;

	std::string ReplaceSpaces(const std::string& str);

};

