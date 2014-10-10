################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Sin256Q15.obj: ../Sin256Q15.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/cl2000" -v28 -ml -mt -O0 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.8/include" --include_path="C:/ti/controlSUITE/development_kits/C2000_LaunchPad" --include_path="C:/ti/xdais_7_24_00_04/xdais_7_24_00_04/packages/ti/xdais" -g --define="_FLASH" --define=NDEBUG --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --gen_func_subsections=on --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="Sin256Q15.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

VoiceModule.obj: ../VoiceModule.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"c:/ti/ccsv6/tools/compiler/c2000_6.2.8/bin/cl2000" -v28 -ml -mt -O0 --include_path="c:/ti/ccsv6/tools/compiler/c2000_6.2.8/include" --include_path="C:/ti/controlSUITE/development_kits/C2000_LaunchPad" --include_path="C:/ti/xdais_7_24_00_04/xdais_7_24_00_04/packages/ti/xdais" -g --define="_FLASH" --define=NDEBUG --define="_DEBUG" --define="LARGE_MODEL" --quiet --verbose_diagnostics --diag_warning=225 --gen_func_subsections=on --output_all_syms --cdebug_asm_data --preproc_with_compile --preproc_dependency="VoiceModule.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


