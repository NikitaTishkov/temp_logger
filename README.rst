#################################
Temprature logger
#################################
Temprature logger based on STM32F4 DISCOVERY and GlobalLogic Starter Kit
Device capabilities:
    Recording data from analog tempreture sensor to external Flash storage;
    Two tempreture modes (Farenheit - Celcium);
    Trigger mode(recording starts when the temperature is outside the range of acceptable values);
    
Requirements
************

.. note::
   Arch Linux means access to the most recent software versions. Package names and installation
   commands provided here are given for the Arch Linux and its derivatives (i.e. Manjaro).
   
   If you are using another distro, you need to figure the package names yourself or use something
   like `Archlinux Docker image <https://hub.docker.com/_/archlinux>`_.

- `OpenOCD <http://openocd.org>`_.
  
  | Stable version: ``openocd``
    (Not recommended as it is outdated and incompatible with our openocd config.
     If you prefer stable version, use ``openocd -f board/stm32f4discovery.cfg``
     instead of our ``openocd -f openocd_glstarterkit.cfg``)
  | Latest Git version: ``openocd-git`` (through AUR)
- `arm-none-eabi Toolchain <https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm>`_
  
 

License
*******
| Everything in this repository, except the STMicroelectronics documentation is licensed
  under the MIT License.
| See `<LICENSE>`_ for details.
| 
| For more on STMicroelectronics documentation licensing consider their official website
  (`<https://st.com>`_)

Contact information
*******************
Should you have questions, feel free to contact me via Telegram
(`@thodnev <https://t.me/thodnev>`_) or e-mail (thodnev <at> xinity.dev)
