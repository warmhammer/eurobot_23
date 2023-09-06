Файл автозапуска на RPI находится тут: /etc/xdg/lxsession/LXDE-pi/autostart

Если автозапуска не произошло, то надо вбить следующие команды:
```
export DISPLAY=:0
eurobot_23/basket/basket_autorun.sh   # запуск bash скрипта
```

Вероятно, если вы вбиваете комадны не по SSH, то export не нужен.
