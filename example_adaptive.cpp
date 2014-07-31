/*
	Optimized scalable arithmetic encoder/decoder implementation by:
		Dimitris Vlachos(DimitrisV22@gmail.com) , 2014
		(https://github.com/DimitrisVlachos/lib_bitstreams)

	Based on non-scalable basic implementations of :
		Mark Nelson
		Dimitry Subbotin (carry-less implementation of range coder)
		Sachin Garg

	Dependencies :
	Requires my bitstream library 
	https://github.com/DimitrisVlachos/lib_bitstreams

	License :
		MIT
*/

#include "scalable_ac.hpp"
#include "scalable_adc.hpp"


bool encode(const char* in_file,const char* out_file) {
	file_streams::file_stream_if* rd = new file_streams::file_stream_reader_c(in_file);
	bit_streams::bit_stream_writer_c<file_streams::file_stream_writer_c> out;
	scalable_ac_c<file_streams::file_stream_writer_c,uint32_t,uint64_t> coder;

	if (!rd)
		return false;

	if (!out.open(out_file)) {
		delete rd;
		return false;
	}

 	
	coder.init(256 + 1,&out); //256 is eof symbol

	for (uint32_t i = 0,j = rd->size();i < j;++i)
		coder.encode_symbol(rd->read());

	coder.encode_symbol(256); // eof
	coder.flush();
	out.close();
	delete rd;
	return true;
}

bool decode(const char* in_file,const char* out_file) {
	file_streams::file_stream_if* wr = new file_streams::file_stream_writer_c(out_file);
	bit_streams::bit_stream_reader_c<file_streams::file_stream_reader_c> in;
	scalable_adc_c<file_streams::file_stream_reader_c,uint32_t,uint64_t> decoder;

	if (!wr)
		return false;

	if (!in.open(in_file)) {
		delete wr;
		return false;
	}

	decoder.init(256 + 1,&in); //256 is eof symbol
 	uint32_t symbol;

	while (1) {
		symbol = decoder.decode_symbol();
		if (256==symbol)break;
		wr->write(symbol);
	}

	delete wr;
	return true;
}

int main() {
	encode("scalable_ac.hpp","out.bin");
	decode("out.bin","out_scalable_ac.hpp");
	return 0;
}
