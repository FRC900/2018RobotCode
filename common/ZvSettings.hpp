#ifndef ZVSETTINGS_HPP__
#define ZVSETTINGS_HPP__

#include <iostream>
#include <tinyxml2.h>

class ZvSettings
{
public:
    ZvSettings(const std::string &filename);
    bool save();

    bool getInt(const std::string &sectionName,
                const std::string &name,
                int &value);

    bool getUnsignedInt(const std::string &sectionName,
                        const std::string &name,
                        unsigned int &value);

    bool getDouble(const std::string &sectionName,
                   const std::string &name,
                   double &value);

    bool setInt(const std::string &sectionName,
                const std::string &name,
                const int value);

    bool setDouble(const std::string &sectionName,
                   const std::string &name,
                   const double value);

    template <class T>
    void set(const std::string &sectionName,
             const std::string &name,
             const T value);

private:
    tinyxml2::XMLElement *getElement(const std::string &sectionName,
                                     const std::string &name);
    tinyxml2::XMLDocument xmlDoc_;
    std::string filename_;
};

#endif
