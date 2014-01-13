#pragma once

#include <fstream>
#include <string>

class DescriptorBase
{
public:
    virtual ~DescriptorBase() {};
};

template<typename T>
class Descriptor : public DescriptorBase
{
public:
    Descriptor() = default;
    Descriptor(const std::vector<T>& data)
        : m_data(data)
    {}

    ~Descriptor() {}

    inline std::size_t Size() const           { return m_data.size(); }

    inline T& operator[](std::size_t i)       { return m_data[i]; }
    inline T  operator[](std::size_t i) const { return m_data[i]; }

    inline T* GetData() const                 { return m_data.data(); }

private:
    std::vector<T> m_data;
};

template<typename T>
bool SaveDescriptorsToFileBinary(const std::string &filename, const T &descriptors)
{
    std::ofstream file(filename, std::ios::out | std::ios::binary);

    const std::size_t size = descriptors.size();
    file.write(reinterpret_cast<const char*>(&size), sizeof(std::size_t));
    file.write(reinterpret_cast<const char*>(descriptors.data()), size * sizeof(typename T::value_type));

    file.close();
    return file.good();
}

template<typename T>
bool LoadDescriptorsFromFileBinary(const std::string & filename, T &descriptors)
{
    descriptors.clear();
    std::ifstream file(filename, std::ios::in | std::ios::binary);

    std::size_t size;
    file.read(reinterpret_cast<char*>(&size), sizeof(std::size_t));

    descriptors.resize(size);
    file.read(reinterpret_cast<char*>(descriptors.data()), size * sizeof(typename T::value_type));

    file.close();
    return file.good();
}