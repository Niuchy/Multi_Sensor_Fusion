#ifndef _FAST_DDS_GENERATED_MESSAGETYPE_H_
#define _FAST_DDS_GENERATED_MESSAGETYPE_H_

#include <array>
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <fastcdr/cdr/fixed_size_string.hpp>
#include <fastcdr/xcdr/external.hpp>
#include <fastcdr/xcdr/optional.hpp>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec(dllexport)
#else
#define eProsima_user_DllExport
#endif // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(MESSAGETYPE_SOURCE)
#define MESSAGETYPE_DllAPI __declspec(dllexport)
#else
#define MESSAGETYPE_DllAPI __declspec(dllimport)
#endif // MESSAGETYPE_SOURCE
#else
#define MESSAGETYPE_DllAPI
#endif // EPROSIMA_USER_DLL_EXPORT
#else
#define MESSAGETYPE_DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
        class CdrSizeCalculator;
    } // namespace fastcdr
} // namespace eprosima

class MessageType
{
public:
    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport MessageType();

    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~MessageType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object MessageType that will be copied.
     */
    eProsima_user_DllExport MessageType(
        const MessageType &x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object MessageType that will be copied.
     */
    eProsima_user_DllExport MessageType(
        MessageType &&x) noexcept;

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object MessageType that will be copied.
     */
    eProsima_user_DllExport MessageType &operator=(
        const MessageType &x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object MessageType that will be copied.
     */
    eProsima_user_DllExport MessageType &operator=(
        MessageType &&x) noexcept;

    /*!
     * @brief Comparison operator.
     * @param x MessageType object to compare.
     */
    eProsima_user_DllExport bool operator==(
        const MessageType &x) const;

    /*!
     * @brief Comparison operator.
     * @param x MessageType object to compare.
     */
    eProsima_user_DllExport bool operator!=(
        const MessageType &x) const;

    /*!
     * @brief This function copies the value in member message
     * @param _message New value to be copied in member message
     */
    eProsima_user_DllExport void message(
        const std::vector<uint8_t> &_message);

    /*!
     * @brief This function moves the value in member message
     * @param _message New value to be moved in member message
     */
    eProsima_user_DllExport void message(
        std::vector<uint8_t> &&_message);

    /*!
     * @brief This function returns a constant reference to member message
     * @return Constant reference to member message
     */
    eProsima_user_DllExport const std::vector<uint8_t> &message() const;

    /*!
     * @brief This function returns a reference to member message
     * @return Reference to member message
     */
    eProsima_user_DllExport std::vector<uint8_t> &message();

private:
    std::vector<uint8_t> m_message;
};

#endif // _FAST_DDS_GENERATED_MESSAGETYPE_H_
