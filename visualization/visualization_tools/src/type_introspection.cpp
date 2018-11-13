#include <moveit/visualization_tools/type_introspection.h>

#include <iostream>

#include <time.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>

namespace moveit_rviz_plugin {

rviz::Property* TreeConstructor::parseToTree(const ri::RenamedValues& msgValues,
                                             const std::string& name, const std::string& description,
                                             rviz::Property* old) {
    rviz::Property* root;
    if (old) {
        root = old;
        root->setName(QString::fromStdString(name));
        root->setDescription(QString::fromStdString(description));
    } else {
        root = new rviz::StringProperty(QString::fromStdString(name), QString(),
                                        QString::fromStdString(description));
        root->setReadOnly(true);
    }

    if (msgValues.size() == 0)
        return root;

    const size_t startIx = name.size() + 1; // skip the slash '/'
    for (const std::pair<std::string, ri::Variant>& value : msgValues) {
        treeFromValue(value, root, startIx);
    }
    return root;
}

void TreeConstructor::treeFromValue(const std::pair<std::string, ri::Variant>& value,
                                    rviz::Property* root, const size_t startIx) {
    size_t oldStartIx = startIx;
    size_t newStartIx = value.first.find('/', startIx);
    rviz::Property* parent = root;

    if (newStartIx != std::string::npos) {
        newStartIx += 1; // skip the slash '/'
        while (true) { // while (newStartIx != std::string::npos)
            // subtract 2 from newStartIx to get take everything excluding the slash
            parent = getOrCreateChild(value.first.substr(oldStartIx, newStartIx - 2), parent);

            oldStartIx = newStartIx;
            newStartIx = value.first.find('/', newStartIx);
            if (newStartIx != std::string::npos)
                newStartIx += 1; // skip the slash '/'
            else
                break;
        }
    }
    createLeaf(value, parent, oldStartIx);
}

rviz::Property* TreeConstructor::getOrCreateChild(const std::string& name, rviz::Property* parent) {
	rviz::Property* possibleChild = NULL;
	for (int i = 0; i < parent->numChildren(); ++i) {
		if ((possibleChild = parent->childAt(i)) && possibleChild->getNameStd() == name)
			return possibleChild;
	}
	return new rviz::Property(QString::fromStdString(name), QString(), QString(), parent);
}

rviz::Property* TreeConstructor::createLeaf(const std::pair<std::string, ri::Variant>& value,
                                            rviz::Property* parent, const size_t startIx) {
    // The reason for this check is that rviz::IntProperty takes an
    // `int`. We want to check for this (see below).
    constexpr size_t intsize = sizeof(int);

    // Number types are separately handled as strings to avoid showing
    // wrong numbers due to overflow.
    ri::BuiltinType type = value.second.getTypeID();
    switch (type) {
    case ri::BOOL:
        return createLeafOfType<rviz::BoolProperty, bool>(value, parent, startIx);
        /* TODO these are currently not supported in RosTypeIntrospection
        case ri::BYTE:
            return createLeafOfType<rviz::IntProperty, TODO unsigned char>(value, parent, startIx);
        case ri::CHAR:
            return createLeafOfType<rviz::IntProperty, char>(value, parent, startIx);
        */
    case ri::INT8:
        return createLeafOfType<rviz::IntProperty, int8_t>(value, parent, startIx);
    case ri::UINT8:
        if (intsize > 1)
            return createLeafOfType<rviz::IntProperty, uint8_t>(value, parent, startIx);
        else
            return createStringLeaf<uint8_t>(value, parent, startIx);
    case ri::INT16:
        if (intsize >= 2)
            return createLeafOfType<rviz::IntProperty, int16_t>(value, parent, startIx);
        else
            return createStringLeaf<int16_t>(value, parent, startIx);
    case ri::UINT16:
        if (intsize > 2)
            return createLeafOfType<rviz::IntProperty, uint16_t>(value, parent, startIx);
        else
            return createStringLeaf<uint16_t>(value, parent, startIx);
    case ri::INT32:
        if (intsize >= 4)
            return createLeafOfType<rviz::IntProperty, int32_t>(value, parent, startIx);
        else
            return createStringLeaf<int32_t>(value, parent, startIx);
    case ri::UINT32:
        if (intsize > 4)
            return createLeafOfType<rviz::IntProperty, uint32_t>(value, parent, startIx);
        else
            return createStringLeaf<uint32_t>(value, parent, startIx);
    case ri::INT64:
        if (intsize >= 8)
            return createLeafOfType<rviz::IntProperty, int64_t>(value, parent, startIx);
        else
            return createStringLeaf<int64_t>(value, parent, startIx);
    case ri::UINT64:
        if (intsize > 8)
            return createLeafOfType<rviz::IntProperty, uint64_t>(value, parent, startIx);
        else
            return createStringLeaf<uint64_t>(value, parent, startIx);
    case ri::FLOAT32:
        return createLeafOfType<rviz::FloatProperty, float>(value, parent, startIx);
    case ri::FLOAT64:
        return createStringLeaf<double>(value, parent, startIx);
    case ri::STRING:
        return new rviz::StringProperty(QString::fromStdString(value.first.substr(startIx)),
                                        QString::fromStdString(value.second.extract<std::string>()),
                                        QString(), parent);
    case ri::TIME:
        return timeLeaf(value, parent, startIx);
    case ri::DURATION:
        return durationLeaf(value, parent, startIx);
    default:
        throw std::runtime_error("Unsupported type");
    }
    return nullptr;
}

template <typename T, typename V>
rviz::Property* TreeConstructor::createLeafOfType(const std::pair<std::string, ri::Variant>& value,
                                                  rviz::Property* parent, const size_t startIx) {
    rviz::Property* leaf = new T(QString::fromStdString(value.first.substr(startIx)),
                                 value.second.extract<V>(), QString(), parent);
    return leaf;
}

template <typename V>
rviz::StringProperty* TreeConstructor::createStringLeaf(const std::pair<std::string, ri::Variant>& value,
                                                        rviz::Property* parent, const size_t startIx) {
    return new rviz::StringProperty(QString::fromStdString(value.first.substr(startIx)),
                                    QString::fromStdString(std::to_string(value.second.extract<V>())),
                                    QString(), parent);
}

rviz::StringProperty* TreeConstructor::timeLeaf(const std::pair<std::string, ri::Variant>& value,
                                                rviz::Property* parent, const size_t startIx) {
    ros::Time time = value.second.extract<ros::Time>();
    std::ostringstream oss;
    oss << time << " s";
    return new rviz::StringProperty(QString::fromStdString(value.first.substr(startIx)),
                                    QString::fromStdString(oss.str()),
                                    QString("Time in Seconds"), parent);
}

rviz::StringProperty* TreeConstructor::durationLeaf(const std::pair<std::string, ri::Variant>& value,
                                                    rviz::Property* parent, const size_t startIx) {
    ros::Duration duration = value.second.extract<ros::Duration>();
    std::ostringstream oss;
    oss << duration << " s";
    return new rviz::StringProperty(QString::fromStdString(value.first.substr(startIx)),
                                    QString::fromStdString(oss.str()),
                                    QString("Duration in Seconds"), parent);
}

ri::RenamedValues TypeIntrospector::extract(const std::string& msg_id, const std::string& serialized) {
	constexpr uint32_t MAX_ARRAY_SIZE = 10;

	ri::FlatMessage   msgContent;
	ri::RenamedValues msgValues;

	parser.deserializeIntoFlatContainer(msg_id, absl::Span<uint8_t>(reinterpret_cast<uint8_t*>(const_cast<char*>(serialized.data())),
	                                                              serialized.size()),
	                                    &msgContent, MAX_ARRAY_SIZE);
	parser.applyNameTransform(msg_id, msgContent, &msgValues);
	return msgValues;
}

} // namespace moveit_rviz_plugin

