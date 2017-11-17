#include <tinyxml2.h>
#include "ZvSettings.hpp"

using namespace std;
using namespace tinyxml2;

static const char * const TOPLEVEL_NAME = "ZebraVision";

ZvSettings::ZvSettings(const std::string &filename) :
    filename_(filename)
{
    if (xmlDoc_.LoadFile(filename.c_str()) != XML_SUCCESS) {
        cerr << "Failed to open settings file" << endl;
    }
}

bool
ZvSettings::save()
{
    return xmlDoc_.SaveFile(filename_.c_str()) == XML_SUCCESS;
}

bool
ZvSettings::getInt(const std::string &sectionName,
                   const std::string &name,
                   int &value)
{
    int tmpInt;
    XMLElement *elem = getElement(sectionName, name);
    if (elem && elem->QueryIntText(&tmpInt) == XML_SUCCESS) {
        value = tmpInt;
        return true;
    }
	return false;
}

// Horrible hack here - hopefully there's no overflow
bool
ZvSettings::getUnsignedInt(const std::string &sectionName,
                           const std::string &name,
                           unsigned int &value)
{
    int tmpInt;
    XMLElement *elem = getElement(sectionName, name);
    if (elem && elem->QueryIntText(&tmpInt) == XML_SUCCESS) {
        value = tmpInt;
        return true;
    }
	return false;
}

bool
ZvSettings::getDouble(const std::string &sectionName,
                      const std::string &name,
                      double &value)
{
    double tmpDouble;
    XMLElement *elem = getElement(sectionName, name);
    if (elem && elem->QueryDoubleText(&tmpDouble) == XML_SUCCESS) {
        value = tmpDouble;
        return true;
    }
	return false;
}

bool
ZvSettings::setInt(const std::string &sectionName,
                   const std::string &name,
                   const int value)
{
    set(sectionName, name, value);
    return true;
}

bool
ZvSettings::setDouble(const std::string &sectionName,
                      const std::string &name,
                      const double value)
{
    set(sectionName, name, value);
    return true;
}

template <class T>
void
ZvSettings::set(const std::string &sectionName,
                const std::string &name,
                const T value)
{
    XMLElement *elem = getElement(sectionName, name);
    if (elem) {
        elem->SetText(value);
    }
    else {
        cerr << "Failed to set value, name=" << name << " value=" << value << endl;
    }
}

XMLElement*
ZvSettings::getElement(const std::string &sectionName,
                       const std::string &name)
{
    bool saveNeeded = false;
    XMLElement *ret = NULL;
    XMLElement *topElem = xmlDoc_.FirstChildElement(TOPLEVEL_NAME);
    if (!topElem) {
        saveNeeded = true;
        topElem = xmlDoc_.InsertEndChild(xmlDoc_.NewElement(TOPLEVEL_NAME))->ToElement();
    }
    if (topElem) {
        XMLElement *secElem = topElem->FirstChildElement(sectionName.c_str());
        if (!secElem) {
            saveNeeded = true;
            secElem = topElem->InsertEndChild(xmlDoc_.NewElement(sectionName.c_str()))->ToElement();
        }
        if (secElem) {
            ret = secElem->FirstChildElement(name.c_str());
            if (!ret) {
                saveNeeded = true;
                ret = secElem->InsertEndChild(xmlDoc_.NewElement(name.c_str()))->ToElement();
            }
        }
    }
    if (saveNeeded && (xmlDoc_.SaveFile(filename_.c_str()) != XML_SUCCESS)) {
		cerr << "Failed to save settings file: " << filename_ << endl;
    }
    return ret;
}
