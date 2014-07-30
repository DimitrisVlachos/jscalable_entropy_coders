/*
	Optimized scalable arithmetic decoder implementation by:
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
		bit_streams::bit_stream_reader_c<file_streams::file_stream_reader_c> in; //requires my bitstreams lib
		scalable_adc_c<file_streams::file_stream_reader_c,uint16_t,uint32_t> coder; 

		const uint32_t max_entropy = 512; 

		in.open("in");
		coder.init(max_entropy,&in);

		for (i = 0;i < max_entropy;++i)
			std :: cout << coder.decode_symbol() << std::endl;
 
		in.close();

	Example usage of save states(for context switching etc..) :
	scalable_adc_state_t* state = coder.save_state();

		...
	coder.restore_state(state,true); //2nd argument deletes state without the need to call coder.delete_state(state); 
*/

#ifndef __scalable_adc_hpp__
#define __scalable_adc_hpp__

template <class reader_type_c,typename probability_type_t,typename max_range_type_t>
class scalable_adc_c {
	public:
	struct scalable_adc_state_t {
		max_range_type_t high,low;
		max_range_type_t tmp_range;
		max_range_type_t max_syms;
		max_range_type_t code;
		probability_type_t* probability;
	};
	private:
	static const max_range_type_t k_max_bits = sizeof(probability_type_t)<<(probability_type_t)3;
	static const max_range_type_t k_hi_bit = k_max_bits - (max_range_type_t)1;
	static const max_range_type_t k_low_bit = k_max_bits - (max_range_type_t)2;
	static const max_range_type_t k_low_bit_mask = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)2)) - (max_range_type_t)1;
	static const max_range_type_t k_low_bit_val = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)2));
	static const max_range_type_t k_hi_bit_mask = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)1)) - (max_range_type_t)1;
	static const max_range_type_t k_hi_bit_val = ((max_range_type_t)1 << (max_range_type_t)(k_max_bits-(max_range_type_t)1));
	static const max_range_type_t k_max_range =  (max_range_type_t)k_low_bit_mask;
	static const max_range_type_t k_probability_range_mask = (max_range_type_t)( ((probability_type_t)-1)  );

	bit_streams::bit_stream_reader_c<reader_type_c>* m_stream;
	max_range_type_t m_high,m_low;
	max_range_type_t m_tmp_range;
	max_range_type_t m_max_syms;
	max_range_type_t m_code;
	probability_type_t* m_probability;

	public:
	scalable_adc_c() : m_stream(0),m_probability(0) {}
	~scalable_adc_c() { delete[] m_probability; }

	inline probability_type_t* get_model() {
		return m_probability;
	}

	scalable_adc_state_t* save_state() {
		scalable_adc_state_t* state = new scalable_adc_state_t();
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
		state->max_syms = m_max_syms;
		state->code = m_code;
		state->tmp_range = m_tmp_range;
		return state;
	}

	void delete_state(scalable_adc_state_t* state) {
		if (!state)
			return;

		delete[] state->probability;
		delete state;
	}

	bool restore_state(scalable_adc_state_t* state,const bool cleanup) {
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
		m_max_syms = state->max_syms;
		m_code = state->code;
		m_tmp_range = state->tmp_range;

		if (cleanup)
			delete_state(state);

		return true;
	}

	max_range_type_t decode_symbol() {
		max_range_type_t prob = get_current_prob(m_probability[m_max_syms]);
		max_range_type_t sym =  (m_max_syms!=0) ? m_max_syms-1 : 0;

		if (sym && (m_probability[sym] > prob)) {
			do {
			} while (sym && m_probability[--sym] > prob);
		}
		remove_range(sym);

		{
			register probability_type_t* p0;
			register probability_type_t* p1;

			p0 = &m_probability[sym + (max_range_type_t)1];
			p1 = &m_probability[m_max_syms + (max_range_type_t)1];
			if (p0 < p1) {
				do {
					*(p0++) += (probability_type_t)1U;
				} while (p0 < p1);
			}
		}

		if(m_probability[m_max_syms] >= k_max_range)
			scale_model();
		
		return sym;
	}

	bool init(max_range_type_t max_symbols,bit_streams::bit_stream_reader_c<reader_type_c>* stream) {

		if ((!stream) || (!max_symbols))
			return false;

		m_high =  (max_range_type_t)( ((probability_type_t)-1)  );
		m_low=0;
		m_tmp_range=0;
		m_stream = stream;
		 
		delete[] m_probability;
		m_probability = new probability_type_t[max_symbols + 1];
		m_max_syms = max_symbols;

		if (!m_probability)
			return false;
		
		for (max_range_type_t i=(max_range_type_t)0;i <= max_symbols;i++)
			m_probability[i]=i;		

		 
		m_code = 0;
		for (max_range_type_t i = 0;i < k_max_bits;++i) {
			m_code <<= (max_range_type_t)1;
			m_code |= (max_range_type_t)m_stream->read(1);
		}
		
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

		for (max_range_type_t i = (max_range_type_t)m_max_syms+(max_range_type_t)1, j = 0;i <= max_symbols;i++)
			tmp[i]=j++;

		delete[] m_probability;
		m_probability = tmp;
		m_max_syms = max_symbols;

		return true;
	}
	
	private:
	inline max_range_type_t get_current_prob(max_range_type_t range) {
		m_tmp_range = (m_high-m_low)+(max_range_type_t)1;
		return (max_range_type_t)(((((m_code-m_low)+(max_range_type_t)1)*range)-1)/m_tmp_range);
	}

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


	void remove_range(max_range_type_t symbol) {
		const max_range_type_t sym_low = (max_range_type_t)m_probability[symbol];
		const max_range_type_t sym_high  = (max_range_type_t)m_probability[symbol + (max_range_type_t)1];
		const max_range_type_t total_range = (max_range_type_t)m_probability[m_max_syms];

		m_tmp_range = (m_high-m_low)+(max_range_type_t)1;
		m_high = m_low+((m_tmp_range*sym_high)/total_range)-(max_range_type_t)1;
		m_low = m_low+((m_tmp_range*sym_low )/total_range);

		do {
			if((m_high & k_hi_bit_val) == (m_low & k_hi_bit_val)) {}
			else {
				if((m_low & k_low_bit_val) && !(m_high	& k_low_bit_val)) {
					m_code ^= k_low_bit_val;
					m_low &= k_low_bit_mask;
					m_high |= k_low_bit_val;
				}
				else
					return;
			}
			m_low = (m_low	<< (max_range_type_t)1) &	k_probability_range_mask;
			m_high = ((m_high << (max_range_type_t)1) |	(max_range_type_t)1) & k_probability_range_mask;
			m_code = ( (m_code << (max_range_type_t)1) | (max_range_type_t)m_stream->read(1) ) & k_probability_range_mask;
		} while (1);
	}
};

#endif
