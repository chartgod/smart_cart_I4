
#ifndef GLDRIVER_H
#define GLDRIVER_H

#include <string>
#include <vector>

namespace SOSLAB
{
	class GL
	{
	public:
		struct framedata_t
		{
			std::vector<double> angle;
			std::vector<double> distance;
			std::vector<double> pulse_width;
		};

	public:
		GL(std::string &gl_udp_ip, int gl_udp_port, int pc_udp_port);
		GL(std::string &gl_serial_name, uint32_t gl_serial_baudrate);
		~GL();

		std::string GetSerialNum(void);
		void ReadFrameData(GL::framedata_t &frame_data, bool filter_on = true);
		void SetFrameDataEnable(bool framedata_enable);

	private:
		void *gl_;
	};

} /* namespace SOSLAB */

#endif