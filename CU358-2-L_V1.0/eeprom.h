#ifndef _EEPROM_H_
#define _EEPROM_H_
extern int16_t eeReadTempBuffer;
unsigned char* ee_read_bytes(  unsigned char * ee_addr, unsigned char * data, unsigned int dataLength);
void ee_write_bytes(unsigned char* ee_addr, const unsigned char * data, unsigned int dataLength);

#define ee_read_value(var) (*(int16_t*)(ee_read_bytes((unsigned char *)var, (unsigned char*)&eeReadTempBuffer, sizeof(*(var)))))
#define ee_write_value(var, value) {eeReadTempBuffer = value; ee_write_bytes(var, (unsigned char*)&eeReadTempBuffer, sizeof(*(var)));}
#ifdef EEMEM_USE_SHADOW

/*
#define ee_shadow_read_init_value(var, shadow)					shadow = *(typeof(var))(ee_read_bytes(var, &eeReadTempBuffer, sizeof(*(var))))
#define ee_shadow_read_value(var, shadow)						shadow
#define ee_shadow_write_value(var, shadow, value)				{shadow = value; ee_write_bytes(var, &shadow, sizeof(*(var)));}
#define ee_shadow_read_init_byes(ee_var, ram_dst, shadow, size)	{ee_read_bytes(ee_var, shadow, size); memcpy(ram_dst, shadow, size);}
#define ee_shadow_read_byes(ee_var, ram_dst, shadow, size)		memcpy(ram_dst, shadow, size)
#define ee_shadow_write_byes(ee_var, ram_src, shadow, size)		{memcpy(ram_src, shadow, size); ee_write_bytes(ee_var, shadow, size);}
*/
#define ee_shadow_read_init_value(var)					     *(unsigned char*)(ee_read_bytes((unsigned char *)&var, (unsigned char *)&shadow_ ## var, sizeof(var)))//var
#define ee_shadow_read_value(var)					     shadow_##var
#define ee_shadow_write_value(var, value)				{shadow_##var = value; ee_write_bytes((unsigned char*)&var, (const unsigned char*)&shadow_##var, sizeof(var));}
#define ee_shadow_read_init_byes(ee_var, ram_dst, size)	{ee_read_bytes(ee_var, shadow_##ee_var, size); memcpy(ram_dst, shadow_##ee_var, size);}
#define ee_shadow_read_byes(ee_var, ram_dst, size)		memcpy(ram_dst, shadow_##ee_var, size)
#define ee_shadow_write_byes(ee_var, ram_src, size)		{memcpy(ram_src, shadow_##ee_var, size); ee_write_bytes(ee_var, shadow_##ee_var, size);}
#else
#define ee_shadow_read_init_value(var, shadow)					ee_read_value(var)
#define ee_shadow_read_value(var, shadow)						ee_read_value(var)
#define ee_shadow_write_value(var, shadow, value)				ee_write_value(var, value)
#define ee_shadow_read_init_byes(ee_var, ram_dst, shadow, size)	ee_read_bytes(ee_var, ram_dst, size)
#define ee_shadow_read_byes(ee_var, ram_dst, shadow, size)		ee_read_bytes(ee_var, ram_dst, size)
#define ee_shadow_write_byes(ee_var, ram_src, shadow, size)		ee_write_bytes(ee_var, ram_src, size)
#endif

#define ee2memcpy(dst, src, size)	ee_read_bytes(src, dst, size)
#define mem2eecpy(dst, src, size)	ee_write_bytes(dst, src, size)

#endif