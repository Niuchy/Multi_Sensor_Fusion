#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <string>
#include <unordered_map>

extern std::string g_dbPrefix;
extern std::string g_confPrefix;

extern std::string ZH_MISSION_MANAGER_DB_PATH;
extern std::string ZD_MISSION_MANAGER_DB_PATH;
extern std::string ZH_BACKUP_MISSION_MANAGER_DB_PATH;
extern std::string ZD_BACKUP_MISSION_MANAGER_DB_PATH;
extern std::string ZH_MISSION_TARGET_LIB_PATH;

extern std::string Default_Commander_TopicDB_Path;
extern std::string Default_TeamLeader_TopicDB_Path;
extern std::string Default_Soldier_TopicDB_Path;
extern std::string Default_Equipment_TopicDB_Path;

extern std::string Default_Commander_Topic_Config_Path;
extern std::string Default_TeamLeader_Topic_Config_Path;
extern std::string Default_Soldier_Topic_Config_Path;
extern std::string Default_Equipment_Topic_Config_Path;

extern std::string FILE_DUMP_DIR_FILE;
extern std::string FILE_DUMP_DIR_DB;
extern std::string FILE_DUMP_DIR_CONFIG;

extern std::string DB_DIR;

extern std::string RESOURCE_DATABASE_ZH;
extern std::string RESOURCE_DATABASE_ZD;
extern std::string CONFIG_FILE;

extern std::string Default_Commander_ReconnDB_Path;

extern std::string g_userManageDb;
extern std::string g_userManageUserConf;

extern std::string g_logDbPath;

extern std::string Default_Picture_Config_Path;

void SetConfigFilePrefix(std::string &prefix);

extern std::string g_userConfig;

extern std::string INFOMANAGE_TARGETS_DATADB;

extern std::string Default_Msg_Save_Dir;

#endif
