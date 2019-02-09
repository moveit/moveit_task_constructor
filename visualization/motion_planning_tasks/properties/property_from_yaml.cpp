/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#include "property_factory.h"
#include <yaml.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>

namespace mtc = ::moveit::task_constructor;

namespace moveit_rviz_plugin {

/** Implement PropertyFactory::createDefault(), creating an rviz::Property (tree)
 *  from a YAML-serialized string.
 *  As we cannot know the required data type for a field from YAML parsing,
 *  we only distinguish numbers (FloatProperty) and all other YAML scalars (StringProperty).
 */
#if 0
// Try to set numeric or arbitrary scalar value from YAML node. Needs to match old's type.
void setScalarValue(rviz::Property* old, const YAML::Node& node)
{
	if (rviz::FloatProperty* p = dynamic_cast<rviz::FloatProperty*>(old)) {
		// value should be a number. If not throws YAML::BadConversion
		p->setValue(node.as<double>());
		return;
	}
	if (rviz::StringProperty* p = dynamic_cast<rviz::StringProperty*>(old)) {
		// value should be an arbitrary string. If not throws YAML::BadConversion
		p->setValue(QString::fromStdString(node.as<std::string>()));
		return;
	}
	throw YAML::BadConversion();
}

// Update existing old rviz:Property or create a new one from scalar YAML node
rviz::Property* createFromScalar(const QString& name, const QString& description,
                                 const YAML::Node& node, rviz::Property* old)
{
	while (old) {  // reuse existing Property?
		try {
			// try to update value, expecting matching rviz::Property
			setScalarValue(old, node);
			// only if setScalarValue succeeded, also update the rest
			old->setName(name);
			old->setDescription(description);
			return old;
		} catch (const YAML::BadConversion&) {
			break;  // on error, break from loop and create a new property
		}
	}

	// if value is a number, create a FloatProperty
	try { return new rviz::FloatProperty(name, node.as<double>(), description); }
	catch (const YAML::BadConversion&) {}

	// otherwise create a StringProperty
	return new rviz::StringProperty(name, QString::fromStdString(node.as<std::string>()), description);
}

// Create a scalar YAML node with given content value
YAML::Node dummyNode(const std::string& content) {
	YAML::Node dummy;
	dummy=content;
	return dummy;
}

// forward declaration
rviz::Property* createFromNode(const QString& name, const QString& description,
                               const YAML::Node& node, rviz::Property* old);

// Reuse old property (or create new one) as parent for a sequence or map
rviz::Property* createParent(const QString& name, const QString& description, rviz::Property* old)
{
	// don't reuse float or string properties (they are for scalars)
	if (dynamic_cast<rviz::FloatProperty*>(old) ||
	    dynamic_cast<rviz::StringProperty*>(old))
		old = nullptr;
	if (!old)
		old = new rviz::Property(name, description);
	else {
		old->setName(name);
		old->setDescription(description);
	}
	return old;
}

// Hierarchically create property from YAML map node
rviz::Property* createFromMap(const QString& name, const QString& description,
                              const YAML::Node& node, rviz::Property* root)
{
	root = createParent(name, description, root);
	int index = 0;  // current child index in root
	for(YAML::const_iterator it=node.begin(); it!=node.end(); ++it) {
		QString child_name = QString::fromStdString(it->first.as<std::string>());
		int num = root->numChildren();
		// find first child with name >= it->name
		int next = index;
		while (next < num && root->childAt(next)->getName() < child_name)
			++next;
		// and remove all children in range [index, next) at once
		root->removeChildren(index, next-index);
		num = root->numChildren();

		// if names differ, insert a new child, otherwise reuse existing
		rviz::Property *old_child = index < num ? root->childAt(index) : nullptr;
		if (old_child && old_child->getName() != child_name)
			old_child = nullptr;

		rviz::Property *new_child = createFromNode(child_name, "", it->second, old_child);
		if (new_child != old_child)
			root->addChild(new_child, index);
		++index;
	}
	// remove remaining children
	root->removeChildren(index, root->numChildren()-index);
	return root;
}

// Hierarchically create property from YAML sequence node. Items are named [#].
rviz::Property* createFromSequence(const QString& name, const QString& description,
                                   const YAML::Node& node, rviz::Property* root)
{
	root = createParent(name, description, root);
	int index = 0;  // current child index in root
	for (YAML::const_iterator it=node.begin(); it != node.end(); ++it) {
		rviz::Property *old_child = root->childAt(index);  // nullptr for invalid index
		rviz::Property *new_child = createFromNode(QString("[%1]").arg(index), "", *it, old_child);
		if (new_child != old_child)
			root->addChild(new_child, index);
		if (++index >= 10)
			break; // limit number of entries
	}
	// remove remaining children
	root->removeChildren(index, root->numChildren()-index);
	return root;
}

// Create a property from any YAML node.
rviz::Property* createFromNode(const QString& name, const QString& description,
                              const YAML::Node& node, rviz::Property* old)
{
	rviz::Property* result = nullptr;
	switch(node.Type()) {
	case YAML::NodeType::Scalar: result = createFromScalar(name, description, node, old); break;
	case YAML::NodeType::Sequence: result = createFromSequence(name, description, node, old); break;
	case YAML::NodeType::Map: result = createFromMap(name, description, node, old); break;
	case YAML::NodeType::Null: result = createFromScalar(name, description, dummyNode("undefined"), old); break;
	default: result = createFromScalar(name, description, dummyNode("unknown YAML node"), old); break;
	}
	result->setReadOnly(true);
	return result;
}
#endif

inline std::string indent(unsigned int depth) { return std::string(2*depth, ' '); }

rviz::Property* PropertyFactory::createDefault(const std::string& name, const std::string& type,
                                               const std::string& description, const std::string& value,
                                               rviz::Property* old)
{
	QString qname = QString::fromStdString(name);
	QString qdesc = QString::fromStdString(description);

	yaml_parser_t parser;
	yaml_parser_initialize(&parser);
	yaml_parser_set_input_string(&parser, reinterpret_cast<const yaml_char_t*>(value.c_str()), value.size());
	try {
		unsigned int depth=0;
		bool proceed = true;
		while (proceed) {
			yaml_event_t event;
			if (!yaml_parser_parse(&parser, &event)) {
				yaml_event_delete(&event);
				throw std::runtime_error(parser.problem);
				break;
			}

			switch(event.type)
			{
			case YAML_NO_EVENT: std::cout << indent(depth) << "No event!" << std::endl; break;
			/* Stream start/end */
			case YAML_STREAM_START_EVENT: std::cout << indent(depth++) << "STREAM START" << std::endl; break;
			case YAML_STREAM_END_EVENT:   std::cout << indent(--depth) << "STREAM END" << std::endl;   break;
			/* Block delimeters */
			case YAML_DOCUMENT_START_EVENT: std::cout << indent(depth++) << "Start Document" << std::endl; break;
			case YAML_DOCUMENT_END_EVENT:   std::cout << indent(--depth) << "End Document" << std::endl;   break;
			case YAML_SEQUENCE_START_EVENT: std::cout << indent(depth++) << "Start Sequence" << std::endl; break;
			case YAML_SEQUENCE_END_EVENT:   std::cout << indent(--depth) << "End Sequence" << std::endl;   break;
			case YAML_MAPPING_START_EVENT:  std::cout << indent(depth++) << "Start Mapping" << std::endl;  break;
			case YAML_MAPPING_END_EVENT:    std::cout << indent(--depth) << "End Mapping" << std::endl;    break;
			/* Data */
			case YAML_ALIAS_EVENT:  std::cout << indent(depth) << "alias anchor=" << event.data.alias.anchor << std::endl; break;
			case YAML_SCALAR_EVENT: std::cout << indent(depth) << "scalar: " << event.data.scalar.value << std::endl; break;
			}
			proceed = event.type != YAML_STREAM_END_EVENT;
			yaml_event_delete(&event);
		}
		return nullptr;
	} catch (const std::exception &e) {
		std::cout << e.what() << std::endl;
		return nullptr;
	}
}

}
