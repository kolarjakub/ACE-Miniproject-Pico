/* Copyright (c) 2023  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "commands.h"

int frame_data_t::command_is(const char* c)
{
  return !strncmp(command, c, COMMANDS_BUF_IN_SIZE);
}


commands_t::commands_t()
{
  count = 0;
  state = cs_wait_for_command;
  memset(buffer, 0, sizeof(buffer));
  process_command = NULL;
  
  frame.command = buffer;
  frame.text = buffer;
  frame.value = 0;
}


void commands_t::init(void (*process_command_function)(frame_data_t frame))
{
  process_command = process_command_function;
}


void commands_t::process_char(char b)
{
  if (state == cs_wait_for_command && isalpha(b))  { // A command allways starts with a letter
    state = cs_reading_data;
    buffer[0] = b;
    count = 1;
  
  } else if (state == cs_reading_data && b == 0x08)  { // BS (Backspace key received)  
    if (count > 0) {
      count--;
      buffer[count] = 0;  
    }    

  } else if (state == cs_reading_data && (b == 0x0A || b == 0x0D))  { // LF or CR (enter key received)
    // Now we can process the buffer
    if (count != 0) {
      buffer[count] = 0; // Guarantee to null terminate the string
      frame.command = buffer; // The command starts at the begining
      
      // Find the first space to separate the command from the text/value
      frame.text = buffer;
      while(*frame.text) {
        if (*frame.text == ' ' | *frame.text == ':') {  // If the first space found or a ':'
          *frame.text = 0; // Command ends here;
          frame.text++;    // "text" starts here
          while(*frame.text == ' ') frame.text++; // or not, we should skip spaces
          break;  // And we can stop scanning
        }
        frame.text++;
      }
      Serial.print(" frame.text: ");
      Serial.print(frame.text);
      // for the numerical parameter try to get the value from the text
      frame.value = atof((const char*)frame.text);

      if (process_command)         // If "process_command" is not null
        (*process_command)(frame); // Do something with the pair (command, value)      

      // Reset the buffer
      count = 0;
      memset(buffer, 0, sizeof(buffer));
      frame.text = buffer;
    } 
    state = cs_wait_for_command;

  
  } else if (state == cs_reading_data  && count < COMMANDS_BUF_IN_SIZE - 1)  { // A new char can be read
    buffer[count] = b;  // Store byte in the buffer
    count++;

  }
}
