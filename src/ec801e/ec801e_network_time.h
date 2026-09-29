#pragma once

#include <cstdint>
#include <ctime>
#include <limits>
#include <string_view>

// AT+QLTS=1 reports GMT even when its timezone suffix is nonzero. Do NOT
// subtract that suffix or use mktime(), which applies the ESP's local timezone.
// Accept both documented DST layouts: "date,time+zz,dst" and "date,time+zz",dst.
inline bool Ec801EParseNetworkTime(std::string_view value, time_t& timestamp) {
    auto trim = [](std::string_view s) {
        while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
        while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.remove_suffix(1);
        return s;
    };
    auto valid_dst = [&](std::string_view s) {
        s = trim(s);
        if (s.empty() || s.front() != ',') return false;
        s = trim(s.substr(1));
        return s.size() == 1 && s[0] >= '0' && s[0] <= '2';
    };
    value = trim(value);
    if (value.size() < 2 || value.front() != '"') return false;
    const size_t quote = value.find('"', 1);
    if (quote == std::string_view::npos) return false;
    const auto suffix = trim(value.substr(quote + 1));
    if (!suffix.empty() && !valid_dst(suffix)) return false;
    auto date = value.substr(1, quote - 1);
    size_t pos = 0;
    auto number = [&](size_t digits, int& out) {
        if (pos + digits > date.size()) return false;
        out = 0;
        for (size_t i = 0; i < digits; ++i) {
            char c = date[pos++];
            if (c < '0' || c > '9') return false;
            out = out * 10 + c - '0';
        }
        return true;
    };
    auto separator = [&](char c) { return pos < date.size() && date[pos++] == c; };
    int year, month, day, hour, minute, second, zone;
    if (!number(4, year) || !separator('/') || !number(2, month) ||
        !separator('/') || !number(2, day) || !separator(',')) return false;
    while (pos < date.size() && date[pos] == ' ') ++pos;
    if (!number(2, hour) || !separator(':') || !number(2, minute) ||
        !separator(':') || !number(2, second)) return false;
    if (pos >= date.size() || (date[pos] != '+' && date[pos] != '-')) return false;
    ++pos;
    if (!number(2, zone) || zone > 48) return false;
    if (pos != date.size() && (!suffix.empty() || !valid_dst(date.substr(pos)))) return false;
    if (year < 2000 || year > 2099 || month < 1 || month > 12 ||
        hour > 23 || minute > 59 || second > 59) return false;
    auto leap = [](int y) { return y % 4 == 0 && (y % 100 != 0 || y % 400 == 0); };
    constexpr int days_per_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    const int month_days = days_per_month[month - 1] + (month == 2 && leap(year));
    if (day < 1 || day > month_days) return false;
    int64_t days = 0;
    for (int y = 1970; y < year; ++y) days += leap(y) ? 366 : 365;
    for (int m = 1; m < month; ++m) days += days_per_month[m - 1] + (m == 2 && leap(year));
    days += day - 1;
    const int64_t seconds = days * 86400 + hour * 3600 + minute * 60 + second;
    if (seconds > std::numeric_limits<time_t>::max()) return false;
    timestamp = static_cast<time_t>(seconds);
    return true;
}
