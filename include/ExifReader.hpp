#pragma once

#include <string>

const int BytesPerFormat[] = {0,1,1,2,4,8,1,1,2,4,8,4,8};

#define TAG_MAKE                0x010F
#define TAG_MODEL               0x0110
#define TAG_EXIF_OFFSET         0x8769
#define TAG_FOCALLENGTH         0x920A
#define TAG_PIXEL_X_DIMENSION   0xA002
#define TAG_PIXEL_Y_DIMENSION   0xA003
#define TAG_FOCALPLANEXRES      0xA20E
#define TAG_FOCALPLANEYRES      0xA20F
#define TAG_FOCALPLANEUNITS     0xA210
#define TAG_INTEROP_OFFSET      0xA005
#define NUM_FORMATS             12
#define FMT_BYTE                1
#define FMT_USHORT              3
#define FMT_ULONG               4
#define FMT_URATIONAL           5
#define FMT_SBYTE               6
#define FMT_SSHORT              8
#define FMT_SLONG               9
#define FMT_SRATIONAL           10
#define FMT_SINGLE              11
#define FMT_DOUBLE              12

// JPEG markers
#define M_SOF0  0xC0          // Start Of Frame N
#define M_SOF1  0xC1          // N indicates which compression process
#define M_SOF2  0xC2          // Only SOF0-SOF2 are now in common use
#define M_SOF3  0xC3
#define M_SOF5  0xC5          // NB: codes C4 and CC are NOT SOF markers
#define M_SOF6  0xC6
#define M_SOF7  0xC7
#define M_SOF9  0xC9
#define M_SOF10 0xCA
#define M_SOF11 0xCB
#define M_SOF13 0xCD
#define M_SOF14 0xCE
#define M_SOF15 0xCF
#define M_SOI   0xD8          // Start Of Image (beginning of datastream)
#define M_SOS   0xDA          // Start Of Scan (begins compressed data)
#define M_EXIF  0xE1          // Exif marker. Also used for XMP data!
#define M_XMP   0x10E1        // Not a real tag (same value in file as Exif!)

struct Section
{
    unsigned char*  Data;
    int             Type;
    unsigned        Size;
};

class ExifReader
{
public:
    ExifReader(const char* filename);

    ~ExifReader();

    int             m_height = 0;           // Image height from EXIF
    int             m_width = 0;            // Image width from EXIF
    double          m_focal = 0.0;          // Focal length in mm from EXIF
    double          m_focal_plane_x_res;
    double          m_focal_plane_units;
    double          m_ccd_width;
    std::string     m_camera_model;         // Model name
    std::string     m_camera_make;          // Maker name

    bool ReadExifInfo();

private:
    const char* m_filename;
    Section*    m_sections = nullptr;
    int         m_sectionsAllocated = 5;
    int         m_sectionsRead = 0;
    int         m_motorolaOrder = 0;
    int         m_process = 0;
    unsigned    m_largest_exif_offset = 0;

    bool        ReadJpgSections(FILE* infile);
    void        ProcessExif(unsigned char* CharBuf, unsigned int length);       // Process an EXIF marker; Describes all the drivel that most digital cameras include...
    void        ProcessExifDir(unsigned char* DirStart, unsigned char* OffsetBase, unsigned ExifLength, int NestingLevel);  // Process one of the nested EXIF directories
    double      ConvertAnyFormat(void* ValuePtr, int Format);                   // Evaluate number, be it int, rational, or float from directory
    int         Get16m(const void* Short);                                      // Get 16 bits motorola order (always) for jpeg header stuff
    int         Get16u(void* Short);                                            // Convert a 16 bit unsigned value from file's native byte order
    unsigned    Get32u(void* Long);                                             // Convert a 32 bit unsigned value from file's native byte order
    int         Get32s(void* Long);                                             // Convert a 32 bit signed value from file's native byte order
};
