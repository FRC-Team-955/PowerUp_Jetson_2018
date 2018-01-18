#include <error_printing.h>

void ErrorPrinting::print_error(std::string error) {
	std::cerr << "[ ERROR ] " << error << std::endl;
}
