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

/** Implement PropertyFactory::createDefault(), creating an rviz::Property (tree)
 *  from a YAML-serialized string.
 *  As we cannot know the required data type for a field from YAML parsing,
 *  we only distinguish numbers (FloatProperty) and all other YAML scalars (StringProperty).
 */

namespace {

// Event-based YAML parser, creating an rviz::Property tree
// https://www.wpsoftware.net/andrew/pages/libyaml.html
class Parser
{
public:
	static const int YAML_ERROR_EVENT = 255;

	Parser(const std::string& value);
	~Parser();

	rviz::Property* process(const QString& name, const QString& description, rviz::Property* old) const;
	static rviz::Property* createScalar(const QString& name, const QString& description, const QByteArray& value,
	                                    rviz::Property* old);

private:
	// return true if there was no error so far
	bool noError() const { return parser_.error == YAML_NO_ERROR; }
	// parse a single event and return it's type, YAML_ERROR_EVENT on parsing error
	int parse(yaml_event_t& event) const;
	// process events: scalar, start mapping, start sequence
	rviz::Property* process(const yaml_event_t& event, const QString& name, const QString& description,
	                        rviz::Property* old) const;

	inline static QByteArray byteArray(const yaml_event_t& event) {
		assert(event.type == YAML_SCALAR_EVENT);
		return QByteArray::fromRawData(reinterpret_cast<const char*>(event.data.scalar.value), event.data.scalar.length);
	}
	// Try to set value of existing rviz::Property (expecting matching types). Return false on error.
	static bool setValue(rviz::Property* old, const QByteArray& value);

	static rviz::Property* createParent(const QString& name, const QString& description, rviz::Property* old);
	rviz::Property* processMapping(const QString& name, const QString& description, rviz::Property* old) const;
	rviz::Property* processSequence(const QString& name, const QString& description, rviz::Property* old) const;

private:
	mutable yaml_parser_t parser_;
};

Parser::Parser(const std::string& value) {
	yaml_parser_initialize(&parser_);
	yaml_parser_set_input_string(&parser_, reinterpret_cast<const yaml_char_t*>(value.c_str()), value.size());
}

Parser::~Parser() {
	yaml_parser_delete(&parser_);
}

int Parser::parse(yaml_event_t& event) const {
	if (!yaml_parser_parse(&parser_, &event)) {
		yaml_event_delete(&event);
		return YAML_ERROR_EVENT;
	}
	return event.type;
}

// main processing function
rviz::Property* Parser::process(const QString& name, const QString& description, rviz::Property* old) const {
	bool stop = false;
	while (!stop) {
		yaml_event_t event;
		switch (parse(event)) {
			case YAML_ERROR_EVENT:
				return Parser::createScalar(name, description, "YAML error", old);
			case YAML_STREAM_END_EVENT:
				stop = true;
				break;

			case YAML_SEQUENCE_START_EVENT:
			case YAML_MAPPING_START_EVENT:
			case YAML_SCALAR_EVENT:
				return process(event, name, description, old);

			default:
				break;
		}
	}
	// if we get here, there was no content in the yaml stream
	return createScalar(name, description, "undefined", old);
}

// default processing for scalar, start mapping, start sequence events
rviz::Property* Parser::process(const yaml_event_t& event, const QString& name, const QString& description,
                                rviz::Property* old) const {
	switch (event.type) {
		case YAML_SEQUENCE_START_EVENT:
			return processSequence(name, description, old);
		case YAML_MAPPING_START_EVENT:
			return processMapping(name, description, old);
		case YAML_SCALAR_EVENT:
			return createScalar(name, description, byteArray(event), old);
		default:
			break;
	}
	assert(false);  // should not be reached
}

// Try to set numeric or arbitrary scalar value from YAML node. Needs to match old's type.
bool Parser::setValue(rviz::Property* old, const QByteArray& value) {
	if (rviz::FloatProperty* p = dynamic_cast<rviz::FloatProperty*>(old)) {
		bool ok = true;
		double v = value.toDouble(&ok);
		if (ok)
			p->setValue(v);
		return ok;
	}
	if (rviz::StringProperty* p = dynamic_cast<rviz::StringProperty*>(old)) {
		// value should be an arbitrary string. If not throws YAML::BadConversion
		p->setValue(value);
		return true;
	}
	return false;
}

// Update existing old rviz:Property or create a new one from scalar YAML node
rviz::Property* Parser::createScalar(const QString& name, const QString& description, const QByteArray& value,
                                     rviz::Property* old) {
	// try to update value, expecting matching rviz::Property
	if (old && setValue(old, value)) {
		// only if setValue succeeded, also update the rest
		old->setName(name);
		old->setDescription(description);
		return old;
	}

	bool ok = true;
	double v = value.toDouble(&ok);
	if (ok)  // if value is a number, create a FloatProperty
		old = new rviz::FloatProperty(name, v, description);
	else  // otherwise create a StringProperty
		old = new rviz::StringProperty(name, value, description);

	old->setReadOnly(true);
	return old;
}

// Reuse old property (or create new one) as parent for a sequence or map
rviz::Property* Parser::createParent(const QString& name, const QString& description, rviz::Property* old) {
	// don't reuse float or string properties (they are for scalars)
	if (dynamic_cast<rviz::FloatProperty*>(old) || dynamic_cast<rviz::StringProperty*>(old))
		old = nullptr;
	if (!old) {
		old = new rviz::Property(name, QVariant(), description);
		old->setReadOnly(true);
	} else {
		old->setName(name);
		old->setDescription(description);
	}
	return old;
}

// Hierarchically create property from YAML map node
rviz::Property* Parser::processMapping(const QString& name, const QString& description, rviz::Property* root) const {
	root = createParent(name, description, root);
	int index = 0;  // current child index in root
	bool stop = false;
	while (!stop && noError()) {  // parse all map items
		yaml_event_t event;
		switch (parse(event)) {  // parse key
			case YAML_MAPPING_END_EVENT:  // all fine, reached end of mapping
				stop = true;
				break;

			case YAML_SCALAR_EVENT: {  // key
				QByteArray key = byteArray(event);
				int num = root->numChildren();
				// find first child with name >= it->name
				int next = index;
				while (next < num && root->childAt(next)->getName() < key)
					++next;
				// and remove all children in range [index, next) at once
				root->removeChildren(index, next - index);
				num = root->numChildren();

				// if names differ, insert a new child, otherwise reuse existing
				rviz::Property* old_child = index < num ? root->childAt(index) : nullptr;
				if (old_child && old_child->getName() != key)
					old_child = nullptr;

				rviz::Property* new_child = nullptr;
				switch (parse(event)) {  // parse value
					case YAML_MAPPING_START_EVENT:
					case YAML_SEQUENCE_START_EVENT:
					case YAML_SCALAR_EVENT:
						new_child = process(event, key, "", old_child);
						break;
					default:  // all other events are an error
						new_child = createScalar(key, "", parser_.problem, old_child);
						root->setValue("YAML error");
						break;
				}

				if (new_child != old_child)
					root->addChild(new_child, index);
				++index;
				break;
			}

			default:  // unexpected event
				root->setValue("YAML error");
				stop = true;
				break;
		}
	}
	// remove remaining children
	root->removeChildren(index, root->numChildren() - index);
	return root;
}

// Hierarchically create property from YAML sequence node. Items are named [#].
rviz::Property* Parser::processSequence(const QString& name, const QString& description, rviz::Property* root) const {
	root = createParent(name, description, root);
	int index = 0;  // current child index in root
	bool stop = false;
	while (!stop && noError()) {  // parse all map items
		yaml_event_t event;
		switch (parse(event)) {
			case YAML_SEQUENCE_END_EVENT:  // all fine, reached end of sequence
				stop = true;
				break;

			case YAML_MAPPING_START_EVENT:
			case YAML_SEQUENCE_START_EVENT:
			case YAML_SCALAR_EVENT: {
				rviz::Property* old_child = root->childAt(index);  // nullptr for invalid index
				rviz::Property* new_child = process(event, QString("[%1]").arg(index), "", old_child);
				if (new_child != old_child)
					root->addChild(new_child, index);
				if (++index >= 10)
					stop = true;  // limit number of shown entries
				break;
			}

			default:  // unexpected event
				root->setValue("YAML error");
				stop = true;
				break;
		}
	}
	// remove remaining children
	root->removeChildren(index, root->numChildren() - index);
	return root;
}
}

namespace moveit_rviz_plugin {

rviz::Property* PropertyFactory::createDefault(const std::string& name, const std::string& type,
                                               const std::string& description, const std::string& value,
                                               rviz::Property* old) {
	QString qname = QString::fromStdString(name);
	QString qdesc = QString::fromStdString(description);
	Parser parser(value);
	return parser.process(qname, qdesc, old);
}
}
