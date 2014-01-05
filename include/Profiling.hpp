#pragma once

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <map>
#include <string>
#include <vector>

using namespace std::chrono;

namespace
{

    std::string DurationToHHMMSSMMM(const milliseconds &duration)
    {
        auto hh     = duration_cast<hours>(duration);
        auto mm     = duration_cast<minutes>(duration % 3600000);
        auto ss     = duration_cast<seconds>(duration % 60000);
        auto msec   = duration_cast<milliseconds>(duration % 1000);

        std::stringstream result;
        result << std::setfill('0')
               << std::setw(2)  << hh.count() << ":" << std::setw(2) << mm.count() << ":"
               << std::setw(2)  << ss.count() << "." << std::setw(3) << msec.count()
               << std::setfill(' ');

        return result.str();
    }

}

struct ProfileManager
{
    ProfileManager() = default;

    void Add(const std::string &name, milliseconds duration)
    {
        m_entries[name].push_back(duration);
    }

    std::string Report() const
    {
        std::stringstream report; report.setf(std::ios::left, std::ios::adjustfield);
        report << "Runtime statistics:" << std::endl;
        report << std::string(66, '-') << std::endl
               << std::setw(30) << "Element"
               << std::setw(7)  << "Calls"
               << std::setw(15) << "Duration"
               << std::setw(11) << "Per call"
               << std::setw(3)  << "%" << std::endl
               << std::string(66, '-') << std::endl;

        typedef std::pair<milliseconds, std::string> report_entry;
        std::vector<report_entry> sorted_entries;
        
        milliseconds overall_duration{0};
        for (const auto &entry : m_entries) for (const auto &duration : entry.second) overall_duration += duration;
        for (const auto &entry : m_entries)
        {
            milliseconds entry_duration{0};
            for (const auto &duration : entry.second) entry_duration += duration;

            std::stringstream line; line.setf(std::ios::left, std::ios::adjustfield);
            line    << std::setw(30) << entry.first
                    << std::setw(7)  << entry.second.size()
                    << std::setw(15) << DurationToHHMMSSMMM(entry_duration)
                    << std::setw(11) << entry_duration.count() / entry.second.size()
                    << std::setw(3)  << entry_duration.count() * 100 / overall_duration.count();

            sorted_entries.push_back(report_entry{entry_duration, line.str()});
        }

        auto compare_first = [&](const report_entry &a, const report_entry &b) { return a.first < b.first; };
        std::sort(sorted_entries.begin(), sorted_entries.end(), compare_first);

        for (const auto &entry : sorted_entries) report << entry.second << std::endl;

        report << std::string(66, '-') << std::endl;
        report << std::setw(37) << "Total:" << DurationToHHMMSSMMM(overall_duration) << std::endl;
        report << std::string(66, '-');

        return report.str();
    }

    std::map<std::string, std::vector<milliseconds>> m_entries;
};

struct ScopedTimer
{
    ScopedTimer() = delete;
    ScopedTimer(ProfileManager &mgr, const std::string &name)
        : m_profile_manager(mgr)
        , m_name(name)
    {
        m_start = high_resolution_clock::now();
    }

    ~ScopedTimer()
    {
        m_profile_manager.Add(m_name, duration_cast<milliseconds>(high_resolution_clock::now() - m_start));
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

    ProfileManager&                    m_profile_manager;
    std::string                        m_name;
    high_resolution_clock::time_point  m_start;
};
