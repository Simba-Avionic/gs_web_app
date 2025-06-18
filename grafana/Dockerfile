FROM grafana/grafana:11.2.3

# Disable Login form or not
ENV GF_AUTH_DISABLE_LOGIN_FORM="true"

# Allow anonymous authentication or not
ENV GF_AUTH_ANONYMOUS_ENABLED="true"

# Role of anonymous user
ENV GF_AUTH_ANONYMOUS_ORG_ROLE="Admin"

# Install plugins here our in your own config file
# ENV GF_INSTALL_PLUGINS=satellogic-3d-globe-panel,vonage-status-panel,pr0ps-trackmap-panel,speakyourcode-button-panel,corpglory-progresslist-panel,yesoreyeram-boomtheme-panel
# ENV GF_INSTALL_PLUGINS="https://github.com/cloudspout/cloudspout-button-panel/releases/download/7.0.23/cloudspout-button-panel.zip;cloudspout-button-panel"

# Add provisioning
ADD ./provisioning /etc/grafana/provisioning

# Add configuration file
ADD ./grafana.ini /etc/grafana/grafana.ini

# Add dashboard json files
ADD ./dashboards /etc/grafana/dashboards

# ADD ./plugins /var/lib/grafana/plugins

CMD ["service", "grafana-server", "restart"]