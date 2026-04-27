//      [ Librerias ]

#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#define PCL_NO_PRECOMPILE

//      [ Estructuras ]

struct Point
{
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
)

typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> EigMatrix;

//      [ Auxiliares ]

std::string Topic_Out;

//      [ Clase Base ]

class Img2Pcl : public rclcpp::Node 
{
    public:
        // Constructor: Se ejecuta al encender el nodo
        Img2Pcl() : Node("Img2Pcl_ROS2") 
        {
            // Recibe los parametros del launch

            //  > Primero se declaran:
            this->declare_parameter<std::string>("topic_in", "/ouster/range_image");
            this->declare_parameter<std::string>("topic_out", "Pruebas_OUT");

            //  > Luego se inicializan:
            std::string Topic_In = this->get_parameter("topic_in").as_string();
            Topic_Out = this->get_parameter("topic_out").as_string();

            // Crear el Publicador
            Publicador = this->create_publisher<sensor_msgs::msg::PointCloud2>(Topic_Out, 10);
            // Crear el Suscriptor
            Suscriptor = this->create_subscription<sensor_msgs::msg::Image>(Topic_In, 10, std::bind(&Img2Pcl::callback_img, this, std::placeholders::_1));
            // Imprime un mensaje en la terminal
            RCLCPP_INFO(this->get_logger(), "Se creo el Nodo");
        }

    private:
        // Es una función que se ejecuta cada vez que surge un cambio en la entrada
        void callback_img(const sensor_msgs::msg::Image::SharedPtr msg) 
        {
            // Declaramos la variable de la Imagen:
            cv_bridge::CvImagePtr Img;
            
            // Tratamos de leer la Imagen:
            try
            {
                Img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
            }
            catch(cv_bridge::Exception& Error)
            {
                RCLCPP_INFO(this->get_logger(), "No se pudo obtener la imagen. Se obtuvo el siguiente error:\n%s", Error.what());
            }

            // Pasamos la imagen a Matriz:
            Eigen::Map<EigMatrix> Matriz(
                reinterpret_cast<uint16_t*>(Img->image.data), 
                Img->image.rows, 
                Img->image.cols
            );

            // Convertimos la Matriz a nube de puntos:
            
            // > Creamos la variable de la nube de puntos:
            PointCloud::Ptr Pcl (new PointCloud);
            // > Reservamos la memoria para la nube de puntos:
            Pcl->points.reserve(Img->image.rows * Img->image.cols);

            // Asignamos el factor para pasar de 16bits a 4mm
            float Factor = (261.0 / 65536.0);
            // Creamos una varable auxiliar de Índice
            int Indice = 0;

            // > Rellenamos la nube de puntos:
            for (int Px = 0; Px < Img->image.rows; Px++)
            {
                for(int Py = 0; Py < Img->image.cols; Py++)
                {
                    // Si el punto es nulo se ignora
                    if(Matriz(Px, Py) == 0) continue;
                    
                    // Pasamos el valor a mm
                    float Valor = Matriz(Px, Py) * Factor;
                    
                    // Declaramos el Punto
                    Point Punto;

                    // Calculamos los ángulos para la transformación
                    float AngH = (22.5 - (45.0 / 128.0) * Px) * (M_PI / 180.0);
                    float AngW = (184.0 - (360.0 / 2048.0) * Py) * (M_PI / 180.0);

                    // Pasamos los valores de rango a XYZ con los ángulos
                    Punto.z = Valor * sin(AngH);
                    Punto.y = sqrt( pow(Valor, 2) - pow(Punto.z, 2) ) * sin(AngW);
                    Punto.x = sqrt( pow(Valor, 2) - pow(Punto.z, 2) ) * cos(AngW);

                    // Añadimos los puntos a la nube
                    Punto.ring = (uint16_t)(Img->image.rows - Px - 1);
                    Punto.intensity = 0.0;                    
                    Pcl->points.push_back(Punto);

                    // Aumentamos el Indice
                    Indice++;
                }
            }
            
            // Convertimos la nube de puntos para publicarla
            sensor_msgs::msg::PointCloud2 PclOut;
            pcl::toROSMsg(*Pcl, PclOut);

            // Configuración de Header para RViz
            PclOut.header.frame_id = "LIDAR_PCL";
            // PCL usa microsegundos para el stamp
            PclOut.header.stamp = msg->header.stamp;

            // Publicamos la nube de puntos:
            this->Publicador->publish(PclOut);
            RCLCPP_INFO(this->get_logger(), "Se publicó la nube a: %s", Topic_Out.c_str());
        }

        // Declaración de un Publicador
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Publicador;

        // Declaración de un Suscriptor
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Suscriptor;
};

//      [ Bucle Principal ]

int main(int argc, char **argv) 
{
    // Inicializa ROS2
    rclcpp::init(argc, argv);
    
    // Crea y activa el nodo
    std::shared_ptr<Img2Pcl> nodo = std::make_shared<Img2Pcl>();
    
    // Mantiene el nodo vivo esperando eventos
    rclcpp::spin(nodo);
    
    // Apaga ROS2 al terminar
    rclcpp::shutdown();
    return 0;
}
