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
	uint32_t probs[256]; 

	if (!rd)
		return false;

	if (!out.open(out_file)) {
		delete rd;
		return false;
	}

	for (uint32_t i = 0;i < 256;++i)
		probs[i] = 0;


	for (uint32_t i = 0,j = rd->size();i < j;++i)
		++probs[rd->read()];

	rd->seek(0);
	out.write(rd->size(),32);

	
	for (uint32_t i = 0;i < 256;++i)	//not optimal ;)
		out.write(probs[i],32);

	
	coder.init<uint32_t>(probs,rd->size(),256,&out);

	for (uint32_t i = 0,j = rd->size();i < j;++i)
		coder.encode_symbol(rd->read());

	coder.flush();
	out.close();
	delete rd;
	return true;
}

bool decode(const char* in_file,const char* out_file) {
	file_streams::file_stream_if* wr = new file_streams::file_stream_writer_c(out_file);
	bit_streams::bit_stream_reader_c<file_streams::file_stream_reader_c> in;
	scalable_adc_c<file_streams::file_stream_reader_c,uint32_t,uint64_t> decoder;
	uint32_t probs[256]; 
	uint32_t rd_size;

	if (!wr)
		return false;

	if (!in.open(in_file)) {
		delete wr;
		return false;
	}

	rd_size = in.read(32);

	for (uint32_t i = 0;i < 256;++i)	//not optimal ;)
		probs[i] = in.read(32);

	decoder.init<uint32_t>(probs,rd_size,256,&in);

	for (uint32_t i = 0;i < rd_size;++i)
		wr->write(decoder.decode_symbol());


	delete wr;
	return true;
}

int main() {
	encode("scalable_ac.hpp","out.bin");
	decode("out.bin","out_scalable_ac.hpp");
	return 0;
}
