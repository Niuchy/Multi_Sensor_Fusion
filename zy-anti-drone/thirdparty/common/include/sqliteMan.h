#ifndef SQLITEDB_H
#define SQLITEDB_H

#include <iostream>
#include <sqlite3.h>
#include <string>
#include <mutex>
#include <vector>
#include "pxsb_common.h"

class SQLiteDB
{
public:
    SQLiteDB(){};

    SQLiteDB(const std::string &dbPath);

    ~SQLiteDB();

    void executeSQL(const std::string &sql);

    void createTable();

    void openDB(const std::string &dbPath);

    void closeDB();

    // 增加数据
    void insertData(const std::string &table, const std::vector<std::pair<std::string, std::string>> &values);

    // 删除数据（根据某个条件）
    void deleteData(const std::string &table, const std::string &condition);

    // 删除所有数据
    void deleteAllData(const std::string &table);

    // 修改数据
    void updateData(const std::string &table, const std::vector<std::pair<std::string, std::string>> &setValues, const std::string &condition);

    // 插入或更新数据
    void insertUpdateData(const std::string table, const std::string &idColumn, const std::string &idValue,
                          const std::vector<std::pair<std::string, std::string>> &values);

    // 查询数据（根据某个条件）
    std::vector<std::vector<std::string>> queryData(const std::string &query);
    std::vector<std::vector<std::string>> queryData(const std::string &table, const std::vector<std::string> &items, const std::string &condition);

private:
    sqlite3 *db;
    mutable std::mutex w_mutex;

    // 这是一个示例回调函数，用于sqlite3_exec。在这个例子中，我们实际上没有使用它，但为了完整性，我还是包括了它。
    static int callback(void *data, int argc, char **argv, char **azColName);
};

#endif
