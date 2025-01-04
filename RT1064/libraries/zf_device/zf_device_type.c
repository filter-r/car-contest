/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          zf_device_type
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

// #include "zf_device_bluetooth_ch9141.h"

#include "zf_device_type.h"


camera_type_enum    camera_type                     = NO_CAMERE;                    // ����ͷ���ͱ���
callback_function   camera_uart_handler             = type_default_callback;        // ����ͨѶ�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת

camera_type_enum    flexio_camera_type              = NO_CAMERE;                    // FLEXIO�ӿ�����ͷ���ͱ���
callback_function   flexio_camera_vsync_handler     = type_default_callback;        // ���жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת
callback_function   flexio_camera_uart_handler      = type_default_callback;        // ����ͨѶ�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת

wireless_type_enum  wireless_type                   = NO_WIRELESS;
callback_function   wireless_module_uart_handler    = type_default_callback;        // ���ߴ��ڽ����жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת
callback_function   wireless_module_spi_handler     = type_default_callback;        // WIFI SPI GPIO�жϺ���ָ�룬���ݳ�ʼ��ʱ���õĺ���������ת

//-------------------------------------------------------------------------------------------------------------------
// �������     �ջص�����
// ����˵��     void        
// ���ز���     void
// ʹ��ʾ��     set_camera_type(CAMERA_GRAYSCALE);
// ��ע��Ϣ     һ���ɸ�����ͷ��ʼ���ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void type_default_callback(void)
{
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ͷ����
// ����˵��     type_set        ѡ��������ͷ����
// ����˵��     vsync_callback  �豸�ĳ��жϻص�����
// ����˵��     dma_callback    �豸�� DMA ����жϻص�����
// ����˵��     uart_callback   �豸�Ĵ��ڻص�����
// ���ز���     void
// ʹ��ʾ��     set_camera_type(CAMERA_GRAYSCALE);
// ��ע��Ϣ     һ���ɸ�����ͷ��ʼ���ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void  set_camera_type (camera_type_enum type_set, callback_function vsync_callback, callback_function dma_callback, callback_function uart_callback)
{
    camera_type = type_set;
    camera_uart_handler = ((uart_callback == NULL) ? (type_default_callback) : (uart_callback));
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ͷ����
// ����˵��     type_set        ѡ��������ͷ����
// ���ز���     void
// ʹ��ʾ��     set_camera_type(CAMERA_GRAYSCALE);
// ��ע��Ϣ     һ���ɸ�����ͷ��ʼ���ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void set_flexio_camera_type (camera_type_enum type_set, callback_function vsync_callback, callback_function dma_callback, callback_function uart_callback)
{
    flexio_camera_type = type_set;
    flexio_camera_uart_handler = ((uart_callback == NULL) ? (type_default_callback) : (uart_callback));
    flexio_camera_vsync_handler = ((vsync_callback == NULL) ? (type_default_callback) : (vsync_callback));
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ģ������
// ����˵��     type_set        	ѡ��������ģ������
// ����˵��     wireless_callback   �豸�Ĵ��ڻص�����
// ���ز���     void
// ʹ��ʾ��     set_wireless_type(WIRELESS_UART);
// ��ע��Ϣ     һ���ɸ�����ģ���ʼ���ڲ�����
//-------------------------------------------------------------------------------------------------------------------
void set_wireless_type (wireless_type_enum type_set, callback_function wireless_callback)
{
    wireless_type = type_set;
    wireless_module_uart_handler = ((wireless_callback == NULL) ? (type_default_callback) : (wireless_callback));
}


