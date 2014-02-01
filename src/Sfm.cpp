#include "Sfm.hpp"

namespace
{

    struct CameraResidual : LMFunctor<double>
    {
        CameraResidual(const Camera& camera, const Point3Vec& points, const Point2Vec& projections, bool adjust_focal)
            : LMFunctor<double>(adjust_focal ? 9 : 6, adjust_focal ? 2 * points.size() + 3 : 2 * points.size())
            , m_camera(camera)
            , m_points(points)
            , m_projections(projections)
            , m_adjust_focal(adjust_focal)
            , m_constrain_focal_weight(0.0001 * points.size())
            , m_constrain_distortion_weight(100.0 * points.size())
        {}

        int operator()(const Vec& x, Vec& fvec) const
        {
            int num_points = static_cast<int>(m_points.size());

            for (int i = 0; i < num_points; ++i)
            {
                Point2 projection = ProjectPoint(x, m_points[i]);

                fvec(2 * i + 0) = m_projections[i].x() - projection.x();
                fvec(2 * i + 1) = m_projections[i].y() - projection.y();
            }

            if (m_adjust_focal)
            {
                fvec(2 * num_points + 0) = m_constrain_focal_weight * (m_camera.m_focal_length - x(6));
                fvec(2 * num_points + 1) = -m_constrain_distortion_weight * x(7);
                fvec(2 * num_points + 2) = -m_constrain_distortion_weight * x(8);
            }

            return 0;
        }

        Point2 ProjectPoint(const Vec& x, const Point3& point) const
        {
            double focal_length = 0.0;
            if (m_adjust_focal) focal_length = x(6);
            else                focal_length = m_camera.m_focal_length;

            // Compute rotation update
            Mat3 Rnew = AngleAxisToRotationMatrix(x.segment(3, 3)) * m_camera.m_R;

            // Project
            Point3 tmp = Rnew * (point - x.head<3>());
            Point2 projection = tmp.head<2>() / -tmp.z();

            if (m_adjust_focal)
            {
                // Apply radial distortion
                double r2 = projection.squaredNorm();
                double factor = 1.0 + x(7) * r2 + x(8) * r2 * r2;   // Taylor expansion for L(r)

                projection *= factor;
            }

            return projection * focal_length;
        }

        Camera      m_camera;
        Point3Vec   m_points;
        Point2Vec   m_projections;

        bool        m_adjust_focal;

        double      m_constrain_focal_weight;
        double      m_constrain_distortion_weight;
    };

}

Mat3 Camera::GetIntrinsicMatrix() const
{
    Mat3 K;
    K.setIdentity();
    K(0, 0) = K(1, 1) = m_focal_length;

    return K;
}

Mat34 Camera::GetProjectionMatrix() const
{
    Mat34 P;
    P << m_R;
    P.col(3) = m_t;

    return GetIntrinsicMatrix() * P;
}

Point2 Camera::Project(const Point3& point) const
{
    // HZ p. 153f
    Point3 tmp = m_R * (point - m_t);
    Point2 projected = tmp.head<2>() / -tmp.z();

    // Compute radial distortion
    double r2 = projected.squaredNorm();
    double factor = 1.0 + m_k[0] * r2 + m_k[1] * r2 * r2;   // Taylor expansion for L(r)

    return projected * factor * m_focal_length;
}

void RefineCamera(Camera* camera, const Point3Vec& points, const Point2Vec& projections, bool adjust_focal)
{
    Vec x{adjust_focal ? 9 : 6};
    if (adjust_focal)   x << camera->m_t.x(), camera->m_t.y(), camera->m_t.z(), 0.0, 0.0, 0.0, camera->m_focal_length, camera->m_k(0), camera->m_k(1);
    else                x << camera->m_t.x(), camera->m_t.y(), camera->m_t.z(), 0.0, 0.0, 0.0;

    CameraResidual functor{*camera, points, projections, adjust_focal};

    Eigen::NumericalDiff<CameraResidual> numDiff{functor};
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraResidual>> lm{numDiff};

    lm.parameters.ftol   = 1.0e-12;
    lm.parameters.xtol   = 1.0e-12;
    lm.parameters.maxfev = 200;

    auto status = lm.minimize(x);

    // Copy out the parameters
    camera->m_t = x.head<3>();
    camera->m_R = AngleAxisToRotationMatrix(x.segment(3, 3)) * camera->m_R;

    if (adjust_focal)
    {
        camera->m_focal_length  = x(6);
        camera->m_k(0)          = x(7);
        camera->m_k(1)          = x(8);
    }
}
