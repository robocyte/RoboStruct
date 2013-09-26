#include "wx/log.h"

#include "ExifReader.hpp"

ExifReader::ExifReader(const char* filename)
	: m_height(0)
	, m_width(0)
	, m_focal(0.0)
	, m_camera_model()
	, m_camera_make()
	, m_filename(filename)
	, m_sections(nullptr)
	, m_sectionsAllocated(5)
	, m_sectionsRead(0)
	, m_motorolaOrder(0)
	, m_process(0)
	, m_largest_exif_offset(0)
{
	m_sections = (Section*)malloc(sizeof(Section) * 5);
}

ExifReader::~ExifReader()
{
	free(m_sections);
}

bool ExifReader::ReadExifInfo()
{
	FILE *infile = fopen(m_filename, "rb");

	if (infile == nullptr)			return false;
	if (!ReadJpgSections(infile))	return false;

	for (int b = 0; b < m_sectionsRead; b++) free(m_sections[b].Data);

	m_sectionsRead = 0;
	fclose(infile);
	return true;
}

bool ExifReader::ReadJpgSections(FILE *infile)
{
	int a = fgetc(infile);
	int data_precision;

	if (a != 0xff || fgetc(infile) != M_SOI) return false;

	for(;;)
	{
		int itemlen;
		int marker = 0;
		int ll,lh, got;
		unsigned char *Data;

		if (m_sectionsRead > m_sectionsAllocated)
		{
			wxLogMessage("Error: Allocation screwup");
			return false;
		}

		if (m_sectionsRead >= m_sectionsAllocated)
		{
			m_sectionsAllocated += m_sectionsAllocated / 2;
			m_sections = (Section*)realloc(m_sections, sizeof(Section) * m_sectionsAllocated);
			
			if (m_sections == nullptr)
			{
				wxLogMessage("Error: Could not allocate data for entire image");
				return false;
			}
		}

		for (a = 0; a <= 16; a++)
		{
			marker = fgetc(infile);
			if (marker != 0xff) break;
			if (a >= 16)
			{
				wxLogMessage("Error: too many padding bytes\n");
				return false;
			}
		}

		m_sections[m_sectionsRead].Type = marker;

		// Read the length of the section.
		lh = fgetc(infile);
		ll = fgetc(infile);

		itemlen = (lh << 8) | ll;

		if (itemlen < 2)
		{
			wxLogMessage("Error: Invalid marker");
			return false;
		}

		m_sections[m_sectionsRead].Size = itemlen;

		Data = (unsigned char *)malloc(itemlen);
		if (Data == nullptr)
		{
			wxLogMessage("Error: Could not allocate memory");
			return false;
		}
		m_sections[m_sectionsRead].Data = Data;

		// Store first two pre-read bytes.
		Data[0] = (unsigned char)lh;
		Data[1] = (unsigned char)ll;

		got = fread(Data + 2, 1, itemlen - 2, infile); // Read the whole section.
		if (got != itemlen - 2)
		{
			wxLogMessage("Error: Premature end of file?");
			return false;
		}
		m_sectionsRead += 1;

		switch(marker)
		{
		case M_SOS:	return true;	// Stop before hitting compressed data 
		case M_EXIF:				// There can be differents section using the same marker.
		{
			if (memcmp(Data + 2, "Exif", 4) == 0)
			{
				ProcessExif(Data, itemlen);
				break;
			} else if (memcmp(Data + 2, "http:", 5) == 0)
			{
				m_sections[m_sectionsRead - 1].Type = M_XMP; // Change tag for internal purposes.
				break;
			}
			break;
		}
		case M_SOF0:
		case M_SOF1:
		case M_SOF2:
		case M_SOF3:
		case M_SOF5:
		case M_SOF6:
		case M_SOF7:
		case M_SOF9:
		case M_SOF10:
		case M_SOF11:
		case M_SOF13:
		case M_SOF14:
		case M_SOF15:
		{
			data_precision = Data[2];
			m_height = Get16m(Data + 3);
			m_width = Get16m(Data + 5);
			m_process = marker;
			break;
		}
		default: break;
		}
	}
	return true;
}

void ExifReader::ProcessExif(unsigned char * ExifSection, unsigned int length)
{
	int FirstOffset;

	// Check the EXIF header component
	static unsigned char ExifHeader[] = "Exif\0\0";
	if (memcmp(ExifSection + 2, ExifHeader, 6))
	{
		wxLogMessage("Incorrect Exif header");
		return;
	}
	
	if (memcmp(ExifSection + 8, "II", 2) == 0)
	{
		m_motorolaOrder = 0;
	} else
	{
		if (memcmp(ExifSection + 8, "MM", 2) == 0)
		{
			m_motorolaOrder = 1;
		} else
		{
			wxLogMessage("Invalid Exif alignment marker");
			return;
		}
	}

	// Check the next value for correctness
	if (Get16u(ExifSection + 10) != 0x2a)
	{
		wxLogMessage("Invalid Exif start");
		return;
	}

	FirstOffset = Get32u(ExifSection + 12);
	if (FirstOffset < 8 || FirstOffset > 16) wxLogMessage("Suspicious offset of first IFD value");

	// First directory starts 16 bytes in. All offsets are relative to 8 bytes in
	ProcessExifDir(ExifSection + 8 + FirstOffset, ExifSection + 8, length - 8, 0);

	// Compute the CCD width in milimeters
	if (m_focal_plane_x_res != 0) m_ccd_width = (m_width * m_focal_plane_units / m_focal_plane_x_res);
}

void ExifReader::ProcessExifDir(unsigned char *DirStart, unsigned char *OffsetBase, unsigned ExifLength, int NestingLevel)
{
	int de, NumDirEntries;
	char IndentString[25];

	if (NestingLevel > 4)
	{
		wxLogMessage("Maximum directory nesting exceeded (corrupt exif header)");
		return;
	}

	memset(IndentString, ' ', 25);
	IndentString[NestingLevel * 4] = '\0';

	NumDirEntries = Get16u(DirStart);
	unsigned char * DirEnd = (DirStart + 2 + 12 * NumDirEntries);
	if (DirEnd + 4 > (OffsetBase + ExifLength))
	{
		if (DirEnd + 2 == OffsetBase + ExifLength || DirEnd == OffsetBase + ExifLength)
		{
		} else
		{
			wxLogMessage("Illegally sized exif subdirectory");
			return;
		}
	}

	for (de = 0; de < NumDirEntries; de++)
	{
		int ByteCount;
		unsigned char *ValuePtr;
		unsigned char *DirEntry = (DirStart + 2 + 12 * de);

		int Tag			= Get16u(DirEntry);
		int Format		= Get16u(DirEntry + 2);
		int Components	= Get32u(DirEntry + 4);

		if ((Format - 1) >= NUM_FORMATS)
		{
			wxLogMessage("Illegal number format %d for tag %04x", Format, Tag);
			continue;
		}

		if ((unsigned)Components > 0x10000)
		{
			wxLogMessage("Illegal number of components %d for tag %04x", Components, Tag);
			continue;
		}

		ByteCount = Components * BytesPerFormat[Format];

		if (ByteCount > 4)
		{
			unsigned OffsetVal;
			OffsetVal = Get32u(DirEntry + 8);

			// If its bigger than 4 bytes, the dir entry contains an offset.
			if (OffsetVal + ByteCount > ExifLength)
			{
				wxLogMessage("Illegal value pointer for tag %04x", Tag);
				continue;
			}

			ValuePtr = OffsetBase+OffsetVal;
			if (OffsetVal > m_largest_exif_offset) m_largest_exif_offset = OffsetVal;
		} else
		{
			ValuePtr = DirEntry + 8;
		}

		// Extract useful components of tag
		switch(Tag)
		{
		case TAG_MAKE:
			char tmp_make [32];
			strncpy(tmp_make, (char *)ValuePtr, ByteCount < 31 ? ByteCount : 31);
			m_camera_make = tmp_make;
			break;

		case TAG_MODEL:
			char tmp_model [40];
			strncpy(tmp_model, (char *)ValuePtr, ByteCount < 39 ? ByteCount : 39);
			m_camera_model = tmp_model;
			break;

		case TAG_PIXEL_Y_DIMENSION:
		case TAG_PIXEL_X_DIMENSION:
			{
				// Use largest of height and width to deal with images that have been rotated to portrait format
				int width = (int)ConvertAnyFormat(ValuePtr, Format);
				if (m_width < width) m_width = width;
				break;
			}

		case TAG_FOCALLENGTH:
			m_focal = (double)ConvertAnyFormat(ValuePtr, Format);
			break;

		case TAG_FOCALPLANEXRES:
            m_focal_plane_x_res = ConvertAnyFormat(ValuePtr, Format);
            break;

        case TAG_FOCALPLANEUNITS:
            switch((int)ConvertAnyFormat(ValuePtr, Format))
			{
                case 1: m_focal_plane_units = 25.4; break;	// inches
                case 2: m_focal_plane_units = 25.4; break;	// inches
                case 3: m_focal_plane_units = 10;   break;	// centimeter
                case 4: m_focal_plane_units = 1;    break;	// milimeter
                case 5: m_focal_plane_units = .001; break;	// micrometer
            }
            break;

		case TAG_EXIF_OFFSET:
		case TAG_INTEROP_OFFSET:
			{
				unsigned char * SubdirStart = OffsetBase + Get32u(ValuePtr);
				if (SubdirStart < OffsetBase || SubdirStart > OffsetBase + ExifLength)
				{
					wxLogMessage("Illegal exif or interop ofset directory link");
				} else
				{
					ProcessExifDir(SubdirStart, OffsetBase, ExifLength, NestingLevel+1);
				}
				continue;
			}
			break;
		}
	}
}

double ExifReader::ConvertAnyFormat(void *ValuePtr, int Format)
{
	double Value;
	Value = 0;

	switch(Format)
	{
	case FMT_SBYTE:		Value = *(signed char *)ValuePtr;	break;
	case FMT_BYTE:		Value = *(unsigned char *)ValuePtr;	break;
	case FMT_USHORT:	Value = Get16u(ValuePtr);			break;
	case FMT_ULONG:		Value = Get32u(ValuePtr);			break;
	case FMT_URATIONAL:
	case FMT_SRATIONAL:
	{
		int Num,Den;
		Num = Get32s(ValuePtr);
		Den = Get32s(4 + (char*)ValuePtr);
		if (Den == 0)
		{
			Value = 0;
		} else
		{
			Value = (double)Num / Den;
		}
		break;
	}
	case FMT_SSHORT:	Value = (signed short)Get16u(ValuePtr);	break;
	case FMT_SLONG:		Value = Get32s(ValuePtr);				break;
	case FMT_SINGLE:	Value = (double)*(float *)ValuePtr;		break;
	case FMT_DOUBLE:	Value = *(double *)ValuePtr;			break;
	default:			wxLogMessage("Illegal format code %d", Format);
	}
	return Value;
}

int ExifReader::Get16m(const void *Short)
{
	return (((unsigned char *)Short)[0] << 8) | ((unsigned char *)Short)[1];
}

int ExifReader::Get16u(void *Short)
{
	if (m_motorolaOrder)	return (((unsigned char *)Short)[0] << 8) | ((unsigned char *)Short)[1];
	else					return (((unsigned char *)Short)[1] << 8) | ((unsigned char *)Short)[0];
}

unsigned ExifReader::Get32u(void *Long)
{
	return (unsigned)Get32s(Long) & 0xffffffff;
}

int ExifReader::Get32s(void *Long)
{
	if (m_motorolaOrder)	return ((( char *)Long)[0] << 24) | (((unsigned char *)Long)[1] << 16) | (((unsigned char *)Long)[2] << 8 ) | (((unsigned char *)Long)[3] << 0 );
	else					return ((( char *)Long)[3] << 24) | (((unsigned char *)Long)[2] << 16) | (((unsigned char *)Long)[1] << 8 ) | (((unsigned char *)Long)[0] << 0 );
}
