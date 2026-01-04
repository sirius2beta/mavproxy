#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QUdpSocket>
#include <QByteArray>
#include <QDebug>
#include <mavlink.h>  // Include MAVLink C++ definitions

typedef enum { STATE_PARSE_MAVLINK, STATE_READ_EXTRA } rx_state_t;

typedef struct {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t extra_buf[3];
    uint8_t extra_index;
    rx_state_t state;
} mav_bridge_t;

class MavlinkToUdp : public QObject
{
    Q_OBJECT

public:
    MavlinkToUdp() {
        u1_to_u2 = new mav_bridge_t;  // Allocate memory
        u2_to_u1 = new mav_bridge_t;  // Allocate memory
        u1_to_u2->state = STATE_PARSE_MAVLINK;
        u2_to_u1->state = STATE_PARSE_MAVLINK;

        // Initialize serial port
        serialPort = new QSerialPort(this);
        serialPort->setPortName("COM8");
        serialPort->setBaudRate(QSerialPort::Baud115200);
        serialPort->setDataBits(QSerialPort::Data8);
        serialPort->setParity(QSerialPort::NoParity);
        serialPort->setStopBits(QSerialPort::OneStop);
        serialPort->setFlowControl(QSerialPort::NoFlowControl);

        // Initialize UDP socket
        udpSocket = new QUdpSocket(this);
        udpSocket->bind(QHostAddress::Any, 5001);

        // Open the serial port
        if (serialPort->open(QIODevice::ReadWrite)) {
            qDebug() << "Serial port open success";
        } else {
            qDebug() << "Failed to open serial port:" << serialPort->errorString();
        }

        // Connect to the readyRead signal for serial port
        connect(serialPort, &QSerialPort::readyRead, this, &MavlinkToUdp::readData);

        // Connect to the readyRead signal for UDP socket
        connect(udpSocket, &QUdpSocket::readyRead, this, &MavlinkToUdp::readUdpData);
    }

    ~MavlinkToUdp() {
        if (serialPort->isOpen()) {
            serialPort->close();
        }

        delete u1_to_u2;  // Free memory
        delete u2_to_u1;  // Free memory
    }

    void handle_extra_bytes(uint8_t *b) {
        rssi_tx = b[0];
        rssi_rx = b[1];
        err    = b[2];
        qDebug() << "Extra bytes received, RSSI TX:" << rssi_tx << ", RSSI RX:" << rssi_rx << ", Error:" << err;
    }

private slots:
    void readData() {
        // Read data byte by byte from the serial port
        while (serialPort->bytesAvailable() > 0) {
            uint8_t byte;
            qint64 bytesRead = serialPort->read(reinterpret_cast<char*>(&byte), 1); // Read 1 byte

            if (bytesRead > 0) {
                // Try parsing the MAVLink message
                if (u1_to_u2->state == STATE_PARSE_MAVLINK) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &u1_to_u2->msg, &u1_to_u2->status)) {
                        // If parsing is successful, prepare to send the message over UDP
                        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                        uint16_t len = mavlink_msg_to_send_buffer(buf, &u1_to_u2->msg);

                        // Send the MAVLink message over UDP
                        if (udpSocket->writeDatagram(reinterpret_cast<char*>(buf), len, QHostAddress("192.168.0.252"),14450) == -1) {
                            qDebug() << "Failed to send UDP message:" << udpSocket->errorString();
                        } else {
                            //qDebug() << "MAVLink message sent successfully!";
                        }

                        //u1_to_u2->state = STATE_READ_EXTRA;
                        u1_to_u2->extra_index = 0;
                    }
                } else { // STATE_READ_EXTRA
                    u1_to_u2->extra_buf[u1_to_u2->extra_index++] = byte;
                    if (u1_to_u2->extra_index >= 3) {
                        handle_extra_bytes(u1_to_u2->extra_buf);
                        u1_to_u2->state = STATE_PARSE_MAVLINK;
                    }
                }
            }
        }
    }

    void readUdpData() {
        // Read incoming UDP data
        while (udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort;

            // Read the datagram
            udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);


            // Forward the received UDP data to the serial port
            if(!datagram.isEmpty()){
                if (serialPort->isOpen()) {
                    qint64 bytesWritten = serialPort->write(datagram);
                    if (bytesWritten == -1) {
                        qDebug() << "Failed to write to serial port:" << serialPort->errorString();
                    } else {
                        qDebug() << "senderport:"<<senderPort<<"Forwarded UDP message to serial port";
                    }
                } else {
                    qDebug() << "Serial port is not open!";
                }
            }
        }
    }

private:
    QSerialPort *serialPort;
    QUdpSocket *udpSocket;
    mav_bridge_t *u1_to_u2;
    mav_bridge_t *u2_to_u1;
    volatile uint8_t rssi_rx = 0;
    volatile uint8_t rssi_tx = 0;
    volatile uint8_t err = 0;
};

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    MavlinkToUdp mavlinkToUdp;

    return a.exec();
}

#include "main.moc"
