#include <fstream>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "wx/log.h"

#include "ExifReader.hpp"
#include "ImageData.hpp"
#include "Options.hpp"

#include "daisy/daisy.h"
#include "kutility/image.h"

ImageData::ImageData(const std::string filename, const std::string filename_short)
	: m_filename(filename)
	, m_filename_short(filename_short)
	, m_camera()
	, m_keys()
	, m_descriptors()
	, m_key_flags()
	, m_visible_points()
	, m_visible_keys()
	, m_init_focal(0.0)
	, m_desc_size(0)
	, m_ignore_in_bundle(false)
	, m_descriptors_loaded(false)
	, m_width(0)
	, m_height(0)
	, m_init_focal_mm(0.0)
{
}

bool ImageData::GetExifInfo()
{
	ExifReader er(m_filename.c_str());

	//TODO: ReadExifInfo maybe should take ref to this as argument..
	if (er.ReadExifInfo())
	{
		m_width = er.m_width;
		m_height = er.m_height;
		m_init_focal_mm = er.m_focal;
		m_camera_make = er.m_camera_make;
		m_camera_model = er.m_camera_model;
		return true;
	} else return false;
}

void ImageData::DetectFeatures(const Options& opts)
{
	// Cleanup data structures
	this->ClearDescriptors();
	m_keys.clear();

	std::vector<cv::KeyPoint> keys;
	const auto img = cv::imread(m_filename);
	cv::Mat grey_img;
	cv::cvtColor(img, grey_img, CV_BGR2GRAY);

	switch (opts.feature_type)
	{
	case 0: // Detect YAPE/Daisy features
		{
			cv::YAPE YapeDet(	opts.yape_radius,
								opts.yape_threshold,
								opts.yape_octaves,
								opts.yape_views,
								opts.yape_base_feature_size,
								opts.yape_clustering_distance);
			YapeDet(grey_img, keys);

			daisy DaisyDesc = daisy();
			DaisyDesc.set_image(grey_img.data, m_height, m_width);
			DaisyDesc.set_parameters(	opts.daisy_radius,
										opts.daisy_radius_quantization,
										opts.daisy_angular_quantization,
										opts.daisy_histogram_quantization);

			DaisyDesc.initialize_single_descriptor_mode();

			m_desc_size = DaisyDesc.descriptor_size();
			m_descriptors.reserve(m_keys.size() * m_desc_size);

			// Compute Daisy descriptors at provided locations
			for (const auto &key : keys)
			{
				std::vector<float> descriptor(m_desc_size);
				DaisyDesc.get_descriptor(key.pt.y, key.pt.x, 35, descriptor.data());
				m_descriptors.insert(m_descriptors.end(), descriptor.begin(), descriptor.end());
			}
			break;
		}
	case 1: // Detect SIFT features
		{
			m_desc_size = 128;
			m_descriptors.reserve(keys.size() * m_desc_size);

			cv::SIFT Sift(	opts.sift_det_threshold,
							opts.sift_det_edge_threshold,
							opts.surf_common_octaves,
							opts.sift_common_octave_layers);
			Sift(grey_img, cv::Mat(), keys, m_descriptors);
			break;
		}
	case 2: // Detect SURF features
		{
			if (opts.surf_desc_extended)	m_desc_size = 128;
			else							m_desc_size = 64;
			m_descriptors.reserve(keys.size() * m_desc_size);

			cv::SURF Surf(	opts.surf_det_hessian_threshold,
							opts.surf_common_octaves,
							opts.surf_common_octave_layers,
							opts.surf_desc_extended,
							opts.surf_desc_upright);
			Surf(grey_img, cv::Mat(), keys, m_descriptors);
			break;
		}
	}

	this->ConvertOpenCVKeys(keys, img);
	this->SaveDescriptors(true);
}

void ImageData::SaveDescriptors(bool clear)
{
	if (m_descriptors.empty()) return;

	std::string filename = m_filename;
	filename.replace(filename.find(".jpg"), 4, ".desc");
	std::ofstream file(filename.c_str(), std::ios::binary);

	file.write(reinterpret_cast<char*>(m_descriptors.data()), m_descriptors.size() * sizeof(float));
	file.close();

	if (clear) this->ClearDescriptors();
}

void ImageData::LoadDescriptors()
{
	if (!m_descriptors.empty()) return;

	std::string filename = m_filename;
	filename.replace(filename.find(".jpg"), 4, ".desc");
	std::ifstream file(filename.c_str(), std::ios::binary);

	m_descriptors.resize(m_keys.size() * m_desc_size);

	file.read(reinterpret_cast<char*>(m_descriptors.data()), m_keys.size() * m_desc_size * sizeof(float));
	file.close();
}

void ImageData::ConvertOpenCVKeys(const std::vector<cv::KeyPoint>& keys, const cv::Mat& image)
{
	m_keys.reserve(keys.size());

	float x_correction_factor = 0.5 * m_width;
	float y_correction_factor = 0.5 * m_height - 1.0;

	for (auto &key : keys)
	{
		float x = key.pt.x - x_correction_factor;
		float y = y_correction_factor - key.pt.y;
		
		int xf = static_cast<int>(std::floor(key.pt.x)), yf = static_cast<int>(std::floor(key.pt.y));
		const uchar *ptr = image.ptr<uchar>(yf);

		m_keys.push_back(KeyPoint(x, y, ptr[3 * xf + 2], ptr[3 * xf + 1], ptr[3 * xf]));
	}
}

void ImageData::ConvertFocalPx()
{
	if (m_width > m_height)	m_init_focal = m_width * (m_init_focal_mm / m_ccd_width);
	else					m_init_focal = m_height * (m_init_focal_mm / m_ccd_width);
}

double ImageData::GetInitFocal()
{
	return m_init_focal;
}

int ImageData::GetWidth()
{
	return m_width;
}

int ImageData::GetHeight()
{
	return m_height;
}

void ImageData::ClearDescriptors()
{
	std::vector<float>().swap(m_descriptors);	// STL swap trick
}

void ImageData::SetTracks()
{
	for (unsigned int i = 0; i < m_visible_points.size(); i++)
	{
		int track(m_visible_points[i]);
		int key(m_visible_keys[i]);

		m_keys[key].m_track = track;
	}

	wxLogMessage("[SetTracks] %i tracks set for image %s", m_visible_points.size(), m_filename_short.c_str());
}