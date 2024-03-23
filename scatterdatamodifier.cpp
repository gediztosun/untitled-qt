#include "scatterdatamodifier.h"
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>
#include <QtCore/qrandom.h>
#include <QtWidgets/QComboBox>
#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>



using namespace QtDataVisualization;

class GroundTruth {
public:
    GroundTruth() = default;
    GroundTruth(const GroundTruth &other) :
            acceleration(other.acceleration), velocity(other.velocity), position(other.position),
            angularAcceleration(other.angularAcceleration), angularVelocity(other.angularVelocity),
            distance(other.distance) {}
    GroundTruth &operator=(const GroundTruth &other) {
        if (this == &other)
            return *this;
        acceleration = other.acceleration;
        velocity = other.velocity;
        position = other.position;
        angularAcceleration = other.angularAcceleration;
        angularVelocity = other.angularVelocity;
        distance = other.distance;
        return *this;
    }

    std::vector<std::vector<double>> &acceleration1() { return acceleration; }

    const std::vector<std::vector<double>> &getPosition() const {
        return position;
    }

    void set_acceleration(std::vector<std::vector<double>> &&acceleration) {
        this->acceleration = std::move(acceleration);
    }
    void set_velocity(std::vector<std::vector<double>> &&velocity) { this->velocity = std::move(velocity); }
    void set_position(std::vector<std::vector<double>> &&position) { this->position = std::move(position); }
    void set_angular_acceleration(std::vector<std::vector<double>> &&angular_acceleration) {
        angularAcceleration = std::move(angular_acceleration);
    }
    void set_angular_velocity(std::vector<std::vector<double>> &&angular_velocity) {
        angularVelocity = std::move(angular_velocity);
    }
    void set_distance(std::vector<std::vector<double>> &&distance) { this->distance = std::move(distance); }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & acceleration;
        ar & velocity;
        ar & position;

        ar & angularAcceleration;
        ar & angularVelocity;
        ar & distance;
    }

    std::vector<std::vector<double>> acceleration, velocity, position;
    std::vector<std::vector<double>> angularAcceleration, angularVelocity, distance;
};

class SensorData {
public:
    SensorData() = default;
    SensorData(std::vector<std::vector<double>> &&data, std::vector<double> &&timestamp) :
            data(std::move(data)), timestamp(std::move(timestamp)) {}

    void set_data(std::vector<std::vector<double>> &&data) { this->data = std::move(data); }
    void set_timestamp(std::vector<double> &&timestamp) { this->timestamp = std::move(timestamp); }
    std::vector<std::vector<double>> &data1() { return data; }
    std::vector<double> &timestamp1() { return timestamp; }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & data;
        ar & timestamp;
    }

    std::vector<std::vector<double>> data;
    std::vector<double> timestamp;
};

class IMUMeasurement {
public:
    IMUMeasurement() = default;
    IMUMeasurement(const SensorData &acceleration, const SensorData &angular_velocity) :
            acceleration(acceleration), angularVelocity(angular_velocity) {}

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & acceleration;
        ar & angularVelocity;
    }

    SensorData acceleration, angularVelocity;
};

class Data {
public:
    Data() = default;
    Data(const GroundTruth &ground_truth, const IMUMeasurement &imu_measurements, const SensorData &gnss_measurement,
         const SensorData &li_dar_measurement) :
            groundTruth(ground_truth), IMUMeasurements(imu_measurements), GNSSMeasurement(gnss_measurement),
            LiDARMeasurement(li_dar_measurement) {}
    Data(Data &&other) :
            groundTruth(std::move(other.groundTruth)), IMUMeasurements(std::move(other.IMUMeasurements)),
            GNSSMeasurement(std::move(other.GNSSMeasurement)), LiDARMeasurement(std::move(other.LiDARMeasurement)) {}
    Data &operator=(Data &&other) {
        if (this == &other)
            return *this;
        groundTruth = std::move(other.groundTruth);
        IMUMeasurements = std::move(other.IMUMeasurements);
        GNSSMeasurement = std::move(other.GNSSMeasurement);
        LiDARMeasurement = std::move(other.LiDARMeasurement);
        return *this;
    }
    GroundTruth &ground_truth() { return groundTruth; }
    IMUMeasurement &imu_measurements() { return IMUMeasurements; }
    SensorData &gnss_measurement() { return GNSSMeasurement; }
    SensorData &li_dar_measurement() { return LiDARMeasurement; }

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & groundTruth;
        ar & IMUMeasurements;
        ar & GNSSMeasurement;
        ar & LiDARMeasurement;
    }

    GroundTruth groundTruth;
    IMUMeasurement IMUMeasurements;
    SensorData GNSSMeasurement, LiDARMeasurement;
};


//#define RANDOM_SCATTER // Uncomment this to switch to random scatter

const int numberOfItems = 3600;
const float curveDivider = 3.0f;
const int lowerNumberOfItems = 900;
const float lowerCurveDivider = 0.75f;

ScatterDataModifier::ScatterDataModifier(Q3DScatter *scatter)
        : m_graph(scatter),
          m_fontSize(40.0f),
          m_style(QAbstract3DSeries::MeshSphere),
          m_smooth(true),
          m_itemCount(lowerNumberOfItems),
          m_curveDivider(lowerCurveDivider)
{
    //! [0]
    m_graph->activeTheme()->setType(Q3DTheme::ThemeEbony);
    QFont font = m_graph->activeTheme()->font();
    font.setPointSize(m_fontSize);
    m_graph->activeTheme()->setFont(font);
    m_graph->setShadowQuality(QAbstract3DGraph::ShadowQualitySoftLow);
    m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);
    //! [0]

    //! [2]
    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    series->setMeshSmooth(m_smooth);
    m_graph->addSeries(series);
    //! [2]

    //! [3]
    addData();
    //! [3]
}

ScatterDataModifier::~ScatterDataModifier()
{
    delete m_graph;
}

void ScatterDataModifier::addData()
{
    // Configure the axes according to the data
    //! [4]
    m_graph->axisX()->setTitle("X");

    m_graph->axisY()->setTitle("Y");

    m_graph->axisZ()->setTitle("Z");
    //! [4]

    //! [5]
    Data newg;
    {

        std::ifstream ifs("mydata");
        boost::archive::text_iarchive ia(ifs);

        ia >> newg;

    }
    std::vector<std::vector<double>> position = newg.ground_truth().getPosition();
    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(position.size());
    QScatterDataItem *ptrToDataArray = &dataArray->first();
    //! [5]

#ifdef RANDOM_SCATTER
    for (int i = 0; i < m_itemCount; i++) {
        ptrToDataArray->setPosition(randVector());
        ptrToDataArray++;
    }
#else
    //! [6]

    for (const auto &p : position) {

            ptrToDataArray->setPosition(QVector3D(static_cast<float>(p[1]),static_cast<float>(p[2]),static_cast<float>(p[0])));
            ptrToDataArray++;

    }
    //! [6]
#endif

    //! [7]
    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
    //! [7]
}

//! [8]
void ScatterDataModifier::changeStyle(int style)
{
    QComboBox *comboBox = qobject_cast<QComboBox *>(sender());
    if (comboBox) {
        m_style = QAbstract3DSeries::Mesh(comboBox->itemData(style).toInt());
        if (m_graph->seriesList().size())
            m_graph->seriesList().at(0)->setMesh(m_style);
    }
}

void ScatterDataModifier::setSmoothDots(int smooth)
{
    m_smooth = bool(smooth);
    QScatter3DSeries *series = m_graph->seriesList().at(0);
    series->setMeshSmooth(m_smooth);
}

void ScatterDataModifier::changeTheme(int theme)
{
    Q3DTheme *currentTheme = m_graph->activeTheme();
    currentTheme->setType(Q3DTheme::Theme(theme));
    emit backgroundEnabledChanged(currentTheme->isBackgroundEnabled());
    emit gridEnabledChanged(currentTheme->isGridEnabled());
    emit fontChanged(currentTheme->font());
}

void ScatterDataModifier::changePresetCamera()
{
    static int preset = Q3DCamera::CameraPresetFrontLow;

    m_graph->scene()->activeCamera()->setCameraPreset((Q3DCamera::CameraPreset)preset);

    if (++preset > Q3DCamera::CameraPresetDirectlyBelow)
        preset = Q3DCamera::CameraPresetFrontLow;
}

void ScatterDataModifier::changeLabelStyle()
{
    m_graph->activeTheme()->setLabelBackgroundEnabled(!m_graph->activeTheme()->isLabelBackgroundEnabled());
}

void ScatterDataModifier::changeFont(const QFont &font)
{
    QFont newFont = font;
    newFont.setPointSizeF(m_fontSize);
    m_graph->activeTheme()->setFont(newFont);
}

void ScatterDataModifier::shadowQualityUpdatedByVisual(QAbstract3DGraph::ShadowQuality sq)
{
    int quality = int(sq);
    emit shadowQualityChanged(quality); // connected to a checkbox in main.cpp
}

void ScatterDataModifier::changeShadowQuality(int quality)
{
    QAbstract3DGraph::ShadowQuality sq = QAbstract3DGraph::ShadowQuality(quality);
    m_graph->setShadowQuality(sq);
}

void ScatterDataModifier::setBackgroundEnabled(int enabled)
{
    m_graph->activeTheme()->setBackgroundEnabled((bool)enabled);
}

void ScatterDataModifier::setGridEnabled(int enabled)
{
    m_graph->activeTheme()->setGridEnabled((bool)enabled);
}
//! [8]

void ScatterDataModifier::toggleItemCount()
{
    if (m_itemCount == numberOfItems) {
        m_itemCount = lowerNumberOfItems;
        m_curveDivider = lowerCurveDivider;
    } else {
        m_itemCount = numberOfItems;
        m_curveDivider = curveDivider;
    }
    m_graph->seriesList().at(0)->dataProxy()->resetArray(0);
    addData();
}

QVector3D ScatterDataModifier::randVector()
{

    return QVector3D(
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
            (float)(QRandomGenerator::global()->bounded(100)) / 100.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 100.0f,
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f);
}
