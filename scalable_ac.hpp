#ifndef __scalable_ac_hpp__
#define __scalable_ac_hpp__

/*
	Optimized scalable arithmetic coder implementation by:
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

	Example usage :
		bit_streams::bit_stream_writer_c<file_streams::file_stream_writer_c> out; //requires my bitstreams lib
		scalable_ac_c<file_streams::file_stream_writer_c,uint16_t,uint32_t> coder; 

		const uint32_t max_entropy = 512; 

		out.open("out");
		coder.init(max_entropy,&out);

		for (i = 0;i < max_entropy;++i)
			coder.encode_symbol(i);

		coder.flush();
		out.close();

	Example usage of save states(for context switching etc..) :
	scalable_ac_state_t* state = coder.save_state();

		...
	coder.restore_state(state,true); //2nd argument deletes state without the need to call coder.delete_state(state); 


	Example usage of calculating encoding cost (in bits) :

	scalable_ac_state_t* state = coder.save_state(); //Save state

	cost = coder.estimate_cost<uint8_t>(buffer[offs],len,bit_limit);	//Encoding cost stored

	coder.restore_state(state,true);	//Restore state and delete

	//Work with cost now :)
	
*/

#include "bit_streams.hpp"

template <class writer_type_c,typename probability_type_t,typename max_range_type_t >
class scalable_ac_c {
	public:
	struct scalable_ac_state_t {
		max_range_type_t high,low,underflow_count;
		max_range_type_t tmp_range;
		max_range_type_t max_syms;
		probability_type_t* probability;
		bool flushed;
	};

	private:
	static const max_range_type_t k_max_bits = sizeof(probability_type_t)<<(probability_type_t)3;
	static const max_range_type_t k_hi_bit = k_max_bits - 1;
	static const max_range_type_t k_low_bit = k_max_bits - 2;
	static const max_range_type_t k_low_bit_mask = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)2)) - (max_range_type_t)1;
	static const max_range_type_t k_low_bit_val = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)2));
	static const max_range_type_t k_hi_bit_mask = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)1)) - (max_range_type_t)1;
	static const max_range_type_t k_hi_bit_val = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)1));
	static const max_range_type_t k_max_range =  (max_range_type_t)k_low_bit_mask;
	static const max_range_type_t k_probability_range_mask = (max_range_type_t)( ((probability_type_t)-1)  ) ;
 	

	private:
	bit_streams::bit_stream_writer_c<writer_type_c>* m_stream;
	max_range_type_t m_high,m_low,m_underflow_count;
	max_range_type_t m_tmp_range;
	max_range_type_t m_max_syms;
	probability_type_t* m_probability;
	bool m_flushed;

	public:
	scalable_ac_c() : 	m_stream(0),m_high( (max_range_type_t)( ((probability_type_t)-1)  )),
	m_low(0),m_max_syms(0),
	m_underflow_count(0),
	m_tmp_range(0),m_flushed(false),m_probability(0) { }
	~scalable_ac_c() {
		flush();
		delete[] m_probability;
	}

	scalable_ac_state_t* save_state() {
		scalable_ac_state_t* state = new scalable_ac_state_t();
		if (!state)
			return 0;

		state->probability = new probability_type_t[m_max_syms+1];
		if (!state->probability) {
			delete state;
			return 0;
		}

		for (max_range_type_t i = 0;i <= m_max_syms;++i)
			state->probability[i] = m_probability[i];

		state->high = m_high;
		state->low = m_low;
		state->underflow_count = m_underflow_count;
		state->max_syms = m_max_syms;
		state->flushed = m_flushed;
		state->tmp_range = m_tmp_range;
		return state;
	}

	void delete_state(scalable_ac_state_t* state) {
		if (!state)
			return;

		delete[] state->probability;
		delete state;
	}

	bool restore_state(scalable_ac_state_t* state,const bool cleanup) {
		if (!state)
			return false;

		//Could just assign memory in this case as long as cleanup==true
		if (m_max_syms != state->max_syms) {
			delete[] m_probability;
			m_probability = new probability_type_t[state->max_syms+1];
			if (!m_probability)
				return false;

			m_max_syms = state->max_syms;
		}

		for (max_range_type_t i = 0;i <= m_max_syms;++i)
			m_probability[i] = state->probability[i];

		m_high = state->high;
		m_low = state->low;
		m_underflow_count = state->underflow_count;
		m_max_syms = state->max_syms;
		m_flushed = state->flushed;
		m_tmp_range = state->tmp_range;

		if (cleanup)
			delete_state(state);

		return true;
	}

	inline probability_type_t* get_model() {
		return m_probability;
	}

	bool flush(const bool force = false) {
		if (!m_stream)
			return false;

		if ((!m_flushed) || force) { 
			++m_underflow_count;
			m_stream->write((m_low>>k_low_bit)&((max_range_type_t)1),1);
			const max_range_type_t bstate = ((m_low>>k_low_bit)^((max_range_type_t)1))&1;
			const uint64_t uf_mask = (bstate) ? (((uint64_t)-1)) : (uint64_t)0;

			for (;m_underflow_count >= 64U;m_underflow_count -= 64U) 
				m_stream->write(uf_mask,64U);
					
			if (m_underflow_count)
				m_stream->write(uf_mask,m_underflow_count);

			m_underflow_count=(max_range_type_t)0;


			m_flushed=true;
			return false;
		}

		return false;
	}

	 bool init(max_range_type_t max_symbols,bit_streams::bit_stream_writer_c<writer_type_c>* stream) {
		flush();
		if ((!stream) || (!max_symbols))
			return false;

		m_high=  (max_range_type_t)( ((probability_type_t)-1)  );
		 
		m_low=0;
		m_underflow_count=0;
		m_tmp_range=0;
		m_flushed=false; 
		m_stream = stream;
		 
		delete[] m_probability;
		m_probability = new probability_type_t[max_symbols + 1];
		m_max_syms = max_symbols;

		if (!m_probability)
			return false;
		
		for (max_range_type_t i=(max_range_type_t)0;i <= max_symbols;i++)
			m_probability[i]=i;		

		return true;
	} 

	bool expand(max_range_type_t max_symbols) {
		if (!m_probability)
			return false;
		else if (max_symbols <= m_max_syms)
			return false;

		probability_type_t* tmp = new probability_type_t[max_symbols + 1];
		if (!tmp)
			return false;

		for (max_range_type_t i = (max_range_type_t)0;i <= m_max_syms;i++)
			tmp[i]=m_probability[i];	

		for (max_range_type_t i = (max_range_type_t)m_max_syms+(max_range_type_t)1;i <= max_symbols;i++)
			tmp[i]=i;		

		delete[] m_probability;
		m_probability = tmp;
		m_max_syms = max_symbols;

		return true;
	}

	void encode_symbol(const max_range_type_t s) {
		range_code(s);

		register probability_type_t* p0;
		register probability_type_t* p1;


		p0 = &m_probability[s + (max_range_type_t)1];
		p1 = &m_probability[m_max_syms + (max_range_type_t)1];
		if (p0 < p1) {
			do {
				*(p0++) += (probability_type_t)1U;
			} while (p0 < p1);
		}
 

		if (m_probability[m_max_syms] >= k_max_range)
			scale_model();	
	}

	//Remember to save/restore states!
	template <typename base_t>
	const max_range_type_t estimate_cost(const base_t s) {
		max_range_type_t cost = range_code(s,true);

		register probability_type_t* p0;
		register probability_type_t* p1;


		p0 = &m_probability[s + (max_range_type_t)1];
		p1 = &m_probability[m_max_syms + (max_range_type_t)1];
		if (p0 < p1) {
			do {
				*(p0++) += (probability_type_t)1U;
			} while (p0 < p1);
		}
 

		if (m_probability[m_max_syms] >= k_max_range)
			scale_model();	

		return cost;
	}

	//Remember to save/restore states!
	template <typename base_t>
	const max_range_type_t estimate_cost(const base_t* s,const max_range_type_t count,const max_range_type_t lim = (max_range_type_t)-1) {
		max_range_type_t cost = 0;
		for (max_range_type_t i = 0,j = count;i < j;++i) {
			cost += estimate_cost(*(s++));
			if (cost > lim)
				break;
		}

		return cost;
	}

	private:

	void scale_model() {
		register probability_type_t* p0 = &m_probability[(max_range_type_t)0];
		register probability_type_t* p1 = &m_probability[(max_range_type_t)m_max_syms + (max_range_type_t)1];
		register probability_type_t prev = *(p0++),curr;
		if (p0 >= p1)
			return;

		do {
			curr = *(p0) >> (probability_type_t)1;
			if (curr <= prev)
				curr = prev + (probability_type_t)1;
	
			*(p0++) = curr;
			prev = curr;
		} while (p0 < p1);
	}


	max_range_type_t range_code(max_range_type_t symbol,const bool simulate = false) {
		const max_range_type_t sym_low = (max_range_type_t)m_probability[symbol];
		const max_range_type_t sym_high = (max_range_type_t)m_probability[symbol + (max_range_type_t)1];
		const max_range_type_t max_range = (max_range_type_t)m_probability[m_max_syms];
		max_range_type_t cost = 0;

		m_tmp_range=(m_high-m_low)+(max_range_type_t)1;
		m_high = m_low + ((m_tmp_range*sym_high)/max_range)- (max_range_type_t)1;
		m_low = m_low + ((m_tmp_range*sym_low )/max_range);

		do {
			if ((m_high & k_hi_bit_val)==(m_low & k_hi_bit_val)) {
				cost += m_underflow_count + 1;
				if (!simulate) {
					m_stream->write(m_high>>k_hi_bit,1);
					const max_range_type_t bstate = (m_high>>k_hi_bit)^(max_range_type_t)1;
					const uint64_t uf_mask = (bstate) ? (((uint64_t)-1)) : (uint64_t)0;

				
					for (;m_underflow_count >= 64U;m_underflow_count -= 64U) 
						m_stream->write(uf_mask,64U);
					
					if (m_underflow_count)
						m_stream->write(uf_mask,m_underflow_count);
				}
				m_underflow_count=(max_range_type_t)0;

			} else {
				if((m_low & k_low_bit_val) && !(m_high & k_low_bit_val)) {
					++m_underflow_count;
					m_low	 &=	k_low_bit_mask;
					m_high |=	k_low_bit_val;
				}
				else break;
			}

			m_low = (m_low<<(max_range_type_t)1) &	k_probability_range_mask;
			m_high = ((m_high<<(max_range_type_t)1)|(max_range_type_t)1) & k_probability_range_mask;
		} while (1);
		return cost;
	}
};

#endif
