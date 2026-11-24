/**
 * @file
 * Class used to store a single pxsb parameter: header files
 */

#ifndef __PXSB_CONFVAR_H__
#define __PXSB_CONFVAR_H__

#include <map>
#include <string>

/**
 * Class used to store a single GamingAnywhere parameter.
 */
class pxsbConfVar
{
private:
	std::string data;								 /**< Store the plain-text value */
	std::map<std::string, std::string> mapdata;		 /**< Used to store the parameter map's key and value */
	std::map<std::string, std::string>::iterator mi; /**< Used for iterate throughout the parameter map */
	void clear();

public:
	pxsbConfVar();
	std::string value();
	pxsbConfVar &operator=(const char *value);
	pxsbConfVar &operator=(const std::string value);
	pxsbConfVar &operator=(const pxsbConfVar var);
	std::string &operator[](const char *key);
	std::string &operator[](const std::string key);
	bool haskey(const char *key);
	// iteratively access to the map
	int msize();
	void mreset();
	std::string mkey();
	std::string mvalue();
	std::string mnextkey();
};

#endif /* __PXSB_CONFVAR_H__ */
